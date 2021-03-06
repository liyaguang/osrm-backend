#ifndef OSRM_EXTRACTOR_GUIDANCE_MOTORWAY_HANDLER_HPP_
#define OSRM_EXTRACTOR_GUIDANCE_MOTORWAY_HANDLER_HPP_

#include "extractor/guidance/intersection_handler.hpp"
#include "extractor/guidance/intersection.hpp"
#include "extractor/query_node.hpp"

#include "util/name_table.hpp"
#include "util/node_based_graph.hpp"

#include <vector>

namespace osrm
{
namespace extractor
{
namespace guidance
{

// Intersection handlers deal with all issues related to intersections.
// They assign appropriate turn operations to the TurnOperations.
class MotorwayHandler : public IntersectionHandler
{
  public:
    MotorwayHandler(const util::NodeBasedDynamicGraph &node_based_graph,
                    const std::vector<QueryNode> &node_info_list,
                    const util::NameTable &name_table);
    ~MotorwayHandler() override final;

    // check whether the handler can actually handle the intersection
    bool canProcess(const NodeID nid,
                    const EdgeID via_eid,
                    const Intersection &intersection) const override final;

    // process the intersection
    Intersection operator()(const NodeID nid,
                            const EdgeID via_eid,
                            Intersection intersection) const override final;

  private:
    Intersection fromMotorway(const EdgeID via_edge, Intersection intersection) const;
    Intersection fromRamp(const EdgeID via_edge, Intersection intersection) const;

    Intersection fallback(Intersection intersection) const;
};

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif /*OSRM_EXTRACTOR_GUIDANCE_MOTORWAY_HANDLER_HPP_*/
