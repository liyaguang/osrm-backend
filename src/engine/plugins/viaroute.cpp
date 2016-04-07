#include "engine/plugins/viaroute.hpp"
#include "engine/datafacade/datafacade_base.hpp"
#include "engine/api/route_api.hpp"
#include "engine/status.hpp"

#include "util/for_each_pair.hpp"
#include "util/integer_range.hpp"
#include "util/json_container.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{

ViaRoutePlugin::ViaRoutePlugin(datafacade::BaseDataFacade &facade_, int max_locations_viaroute)
    : BasePlugin(facade_), shortest_path(&facade_, heaps), alternative_path(&facade_, heaps),
      direct_shortest_path(&facade_, heaps), max_locations_viaroute(max_locations_viaroute)
{
}

Status ViaRoutePlugin::HandleRequest(const api::RouteParameters &route_parameters,
                                     util::json::Object &json_result)
{
    TIMER_START(total);
    BOOST_ASSERT(route_parameters.IsValid());

    if (max_locations_viaroute > 0 &&
        (static_cast<int>(route_parameters.coordinates.size()) > max_locations_viaroute))
    {
        return Error("TooBig",
                     "Number of entries " + std::to_string(route_parameters.coordinates.size()) +
                         " is higher than current maximum (" +
                         std::to_string(max_locations_viaroute) + ")",
                     json_result);
    }

    TIMER_START(phantoms);
    auto phantom_node_pairs = GetPhantomNodes(route_parameters);
    if (phantom_node_pairs.size() != route_parameters.coordinates.size())
    {
        return Error("NoSegment", std::string("Could not find a matching segment for coordinate ") +
                                      std::to_string(phantom_node_pairs.size()),
                     json_result);
    }
    BOOST_ASSERT(phantom_node_pairs.size() == route_parameters.coordinates.size());

    auto snapped_phantoms = SnapPhantomNodes(phantom_node_pairs);
    TIMER_STOP(phantoms);

    const bool allow_u_turn_at_via =
        route_parameters.uturns ? *route_parameters.uturns : facade.GetUTurnsDefault();

    TIMER_START(query);
    InternalRouteResult raw_route;
    auto build_phantom_pairs = [&raw_route, allow_u_turn_at_via](const PhantomNode &first_node,
                                                                 const PhantomNode &second_node)
    {
        raw_route.segment_end_coordinates.push_back(PhantomNodes{first_node, second_node});
        auto &last_inserted = raw_route.segment_end_coordinates.back();
        // enable forward direction if possible
        if (last_inserted.source_phantom.forward_segment_id.id != SPECIAL_SEGMENTID)
        {
            last_inserted.source_phantom.forward_segment_id.enabled |= allow_u_turn_at_via;
        }
        // enable reverse direction if possible
        if (last_inserted.source_phantom.reverse_segment_id.id != SPECIAL_SEGMENTID)
        {
            last_inserted.source_phantom.reverse_segment_id.enabled |= allow_u_turn_at_via;
        }
    };
    util::for_each_pair(snapped_phantoms, build_phantom_pairs);

    if (1 == raw_route.segment_end_coordinates.size())
    {
        if (route_parameters.alternatives && facade.GetCoreSize() == 0)
        {
            alternative_path(raw_route.segment_end_coordinates.front(), raw_route);
        }
        else
        {
            direct_shortest_path(raw_route.segment_end_coordinates, raw_route);
        }
    }
    else
    {
        shortest_path(raw_route.segment_end_coordinates, route_parameters.uturns, raw_route);
    }
    TIMER_STOP(query);

    TIMER_START(api);
    // we can only know this after the fact, different SCC ids still
    // allow for connection in one direction.
    if (raw_route.is_valid())
    {
        api::RouteAPI route_api{BasePlugin::facade, route_parameters};
        route_api.MakeResponse(raw_route, json_result);
    }
    else
    {
        auto first_component_id = snapped_phantoms.front().component.id;
        auto not_in_same_component = std::any_of(snapped_phantoms.begin(), snapped_phantoms.end(),
                                                 [first_component_id](const PhantomNode &node)
                                                 {
                                                     return node.component.id != first_component_id;
                                                 });

        if (not_in_same_component)
        {
            return Error("NoRoute", "Impossible route between points", json_result);
        }
        else
        {
            return Error("NoRoute", "No route found between points", json_result);
        }
    }
    TIMER_STOP(api);
    TIMER_STOP(total);

    util::json::Object json_timings;
    json_timings.values["phantoms"] = TIMER_MSEC(phantoms);
    json_timings.values["query"] = TIMER_MSEC(query);
    json_timings.values["api"] = TIMER_MSEC(api);
    json_timings.values["total"] = TIMER_MSEC(total);
    json_result.values["timings"] = json_timings;

    return Status::Ok;
}
}
}
}
