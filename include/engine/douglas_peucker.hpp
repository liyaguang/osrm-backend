#ifndef DOUGLAS_PEUCKER_HPP_
#define DOUGLAS_PEUCKER_HPP_

#include "util/coordinate.hpp"

#include <vector>
#include <iterator>

namespace osrm
{
namespace engine
{
namespace detail
{

// This is derived from the following formular:
// x = b * (1 + lon/180) => dx = b * dlon/180
// y = b * (1 - lat/180) => dy = b * dlat/180
// dx^2 + dy^2 < min_pixel^2
// => dlon^2 + dlat^2 < min_pixel^2 / b^2 * 180^2
inline std::vector<std::uint64_t> generateThreshold(double min_pixel, unsigned number_of_zoomlevels)
{
    std::vector<std::uint64_t> thresholds(number_of_zoomlevels);
    for (unsigned zoom = 0; zoom < number_of_zoomlevels; ++zoom)
    {
        const double shift = (1u << zoom) * 256;
        const double b = shift / 2.0;
        const double pixel_to_deg = 180. / b;
        const std::uint64_t min_deg = min_pixel * pixel_to_deg * COORDINATE_PRECISION;
        thresholds[zoom] = min_deg * min_deg;
    }

    return thresholds;
}

const constexpr std::uint64_t DOUGLAS_PEUCKER_THRESHOLDS[19] = {
    17797851562500, // z0
    4449462890625,  // z1
    1112364667969,  // z2
    278090639649,   // z3
    69522396241,    // z4
    17380467225,    // z5
    4345050889,     // z6
    1086229764,     // z7
    271557441,      // z8
    67881121,       // z9
    16966161,       // z10
    4239481,        // z11
    1058841,        // z12
    264196,         // z13
    66049,          // z14
    16384,          // z15
    4096,           // z16
    1024,           // z17
    256,            // z18
};

const constexpr auto DOUGLAS_PEUCKER_THRESHOLDS_SIZE =
    sizeof(DOUGLAS_PEUCKER_THRESHOLDS) / sizeof(*DOUGLAS_PEUCKER_THRESHOLDS);
} // ns detail

// These functions compute the bitvector of indicating generalized input
// points according to the (Ramer-)Douglas-Peucker algorithm.
//
// Input is vector of pairs. Each pair consists of the point information and a
// bit indicating if the points is present in the generalization.
// Note: points may also be pre-selected*/
std::vector<util::Coordinate> douglasPeucker(std::vector<util::Coordinate>::const_iterator begin,
                                             std::vector<util::Coordinate>::const_iterator end,
                                             const unsigned zoom_level);

// Convenience range-based function
inline std::vector<util::Coordinate> douglasPeucker(const std::vector<util::Coordinate> &geometry,
                                                    const unsigned zoom_level)
{
    return douglasPeucker(begin(geometry), end(geometry), zoom_level);
}
}
}

#endif /* DOUGLAS_PEUCKER_HPP_ */
