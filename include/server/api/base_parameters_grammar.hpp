#ifndef SERVER_API_BASE_PARAMETERS_GRAMMAR_HPP
#define SERVER_API_BASE_PARAMETERS_GRAMMAR_HPP

#include "engine/api/base_parameters.hpp"

#include "engine/bearing.hpp"
#include "engine/hint.hpp"
#include "engine/polyline_compressor.hpp"

#include <boost/optional.hpp>
//#define BOOST_SPIRIT_DEBUG
#include <boost/spirit/include/qi.hpp>

#include <limits>
#include <string>

namespace osrm
{
namespace server
{
namespace api
{

namespace qi = boost::spirit::qi;

template <typename T, char... Fmt> struct no_trailing_dot_policy : qi::real_policies<T>
{
    template <typename Iterator> static bool parse_dot(Iterator &first, Iterator const &last)
    {
        if (first == last || *first != '.')
            return false;

        static const constexpr char fmt[sizeof...(Fmt)] = {Fmt...};

        if (first + sizeof(fmt) < last && std::equal(fmt, fmt + sizeof(fmt), first + 1u))
            return false;

        ++first;
        return true;
    }

    template <typename Iterator> static bool parse_exp(Iterator &, const Iterator &)
    {
        return false;
    }

    template <typename Iterator, typename Attribute>
    static bool parse_exp_n(Iterator &, const Iterator &, Attribute &)
    {
        return false;
    }

    template <typename Iterator, typename Attribute>
    static bool parse_nan(Iterator &, const Iterator &, Attribute &)
    {
        return false;
    }

    template <typename Iterator, typename Attribute>
    static bool parse_inf(Iterator &, const Iterator &, Attribute &)
    {
        return false;
    }
};

struct BaseParametersGrammar : boost::spirit::qi::grammar<std::string::iterator>
{
    using Iterator = std::string::iterator;
    using RadiusesT = std::vector<boost::optional<double>>;
    using json_policy = no_trailing_dot_policy<double, 'j', 's', 'o', 'n'>;

    BaseParametersGrammar(qi::rule<Iterator> &root_rule_, engine::api::BaseParameters &parameters_)
        : BaseParametersGrammar::base_type(root_rule_), base_parameters(parameters_)
    {
        const auto add_bearing =
            [this](boost::optional<boost::fusion::vector2<short, short>> bearing_range) {
                boost::optional<engine::Bearing> bearing;
                if (bearing_range)
                {
                    bearing = engine::Bearing{boost::fusion::at_c<0>(*bearing_range),
                                              boost::fusion::at_c<1>(*bearing_range)};
                }
                base_parameters.bearings.push_back(std::move(bearing));
            };
        const auto set_radiuses = [this](RadiusesT radiuses) {
            base_parameters.radiuses = std::move(radiuses);
        };
        const auto add_hint = [this](const std::string &hint_string) {
            if (hint_string.size() > 0)
            {
                base_parameters.hints.push_back(engine::Hint::FromBase64(hint_string));
            }
        };
        const auto add_coordinate = [this](const boost::fusion::vector<double, double> &lonLat) {
            base_parameters.coordinates.emplace_back(util::Coordinate(
                util::FixedLongitude(boost::fusion::at_c<0>(lonLat) * COORDINATE_PRECISION),
                util::FixedLatitude(boost::fusion::at_c<1>(lonLat) * COORDINATE_PRECISION)));
        };
        const auto polyline_to_coordinates = [this](const std::string &polyline) {
            base_parameters.coordinates = engine::decodePolyline(polyline);
        };

        alpha_numeral = +qi::char_("a-zA-Z0-9");
        polyline_chars = qi::char_("a-zA-Z0-9_.--[]{}@?|\\%~`^");
        base64_char = qi::char_("a-zA-Z0-9--_=");

        unlimited.add("unlimited", std::numeric_limits<double>::infinity());

        radiuses_rule = qi::lit("radiuses=") > -(unlimited | qi::double_) % ";";
        hints_rule =
            qi::lit("hints=") >
            qi::as_string[qi::repeat(engine::ENCODED_HINT_SIZE)[base64_char]][add_hint] % ";";
        bearings_rule =
            qi::lit("bearings=") > (-(qi::short_ > ',' > qi::short_))[add_bearing] % ";";
        polyline_rule = qi::as_string[qi::lit("polyline(") > +polyline_chars > qi::lit(")")]
                                     [polyline_to_coordinates];
        location_rule = (double_ > qi::lit(',') > double_)[add_coordinate];
        query_rule = (location_rule % ';') | polyline_rule;

        base_rule = bearings_rule | radiuses_rule[set_radiuses] | hints_rule;
    }

  protected:
    qi::rule<Iterator> base_rule;
    qi::rule<Iterator> query_rule;

  private:
    engine::api::BaseParameters &base_parameters;
    qi::rule<Iterator> bearings_rule;
    qi::rule<Iterator> hints_rule;
    qi::rule<Iterator> polyline_rule, location_rule;
    qi::symbols<char, double> unlimited;
    qi::rule<Iterator, RadiusesT()> radiuses_rule;
    qi::rule<Iterator, unsigned char()> base64_char;
    qi::rule<Iterator, std::string()> alpha_numeral, polyline_chars;
    qi::real_parser<double, json_policy> double_;
};
}
}
}

#endif
