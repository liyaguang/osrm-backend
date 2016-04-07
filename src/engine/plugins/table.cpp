#include "engine/plugins/table.hpp"

#include "engine/api/table_parameters.hpp"
#include "engine/api/table_api.hpp"
#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/search_engine_data.hpp"
#include "util/string_util.hpp"
#include "util/timing_util.hpp"
#include "util/json_container.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <boost/assert.hpp>

namespace osrm
{
namespace engine
{
namespace plugins
{

TablePlugin::TablePlugin(datafacade::BaseDataFacade &facade, const int max_locations_distance_table)
    : BasePlugin{facade}, distance_table(&facade, heaps),
      max_locations_distance_table(max_locations_distance_table)
{
}

Status TablePlugin::HandleRequest(const api::TableParameters &params, util::json::Object &result)
{
    TIMER_START(total);
    BOOST_ASSERT(params.IsValid());

    if (!CheckAllCoordinates(params.coordinates))
    {
        return Error("InvalidOptions", "Coordinates are invalid", result);
    }

    if (params.bearings.size() > 0 && params.coordinates.size() != params.bearings.size())
    {
        return Error("InvalidOptions", "Number of bearings does not match number of coordinates",
                     result);
    }

    // Empty sources or destinations means the user wants all of them included, respectively
    // The ManyToMany routing algorithm we dispatch to below already handles this perfectly.
    const auto num_sources =
        params.sources.empty() ? params.coordinates.size() : params.sources.size();
    const auto num_destinations =
        params.destinations.empty() ? params.coordinates.size() : params.destinations.size();

    if (max_locations_distance_table > 0 &&
        ((num_sources * num_destinations) >
         static_cast<std::size_t>(max_locations_distance_table * max_locations_distance_table)))
    {
        return Error("TooBig", "Too many table coordinates", result);
    }

    TIMER_START(phantoms);
    auto snapped_phantoms = SnapPhantomNodes(GetPhantomNodes(params));
    TIMER_STOP(phantoms);
    TIMER_START(query);
    auto result_table = distance_table(snapped_phantoms, params.sources, params.destinations);
    TIMER_STOP(query);

    if (result_table.empty())
    {
        return Error("NoTable", "No table found", result);
    }

    TIMER_START(api);
    api::TableAPI table_api{facade, params};
    table_api.MakeResponse(result_table, snapped_phantoms, result);
    TIMER_STOP(api);
    TIMER_STOP(total);

    util::json::Object json_timings;
    json_timings.values["phantoms"] = TIMER_MSEC(phantoms);
    json_timings.values["query"] = TIMER_MSEC(query);
    json_timings.values["api"] = TIMER_MSEC(api);
    json_timings.values["total"] = TIMER_MSEC(total);
    result.values["timings"] = json_timings;

    return Status::Ok;
}
}
}
}
