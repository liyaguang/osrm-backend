#include <boost/test/unit_test.hpp>
#include <boost/test/test_case_template.hpp>

#include "args.hpp"
#include "fixture.hpp"

#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"
#include "osrm/status.hpp"
#include "osrm/osrm.hpp"

BOOST_AUTO_TEST_SUITE(trip)

BOOST_AUTO_TEST_CASE(test_trip)
{
    const auto args = get_args();
    BOOST_REQUIRE_EQUAL(args.size(), 1);

    using namespace osrm;

    auto osrm = getOSRM(args[0]);

    /*
    TripParameters params;

    json::Object result;

    const auto rc = osrm.Trip(params, result);

    BOOST_CHECK(rc == Status::Ok || rc == Status::Error);
    */
}

BOOST_AUTO_TEST_SUITE_END()
