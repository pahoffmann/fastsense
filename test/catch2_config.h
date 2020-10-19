/**
 * @author Marcel Flottmann
 */

#define CATCH_CONFIG_FAST_COMPILE

#include <catch2/catch.hpp>
#include <algorithm>

template<class C, typename D>
void REQUIRE_EACH(const C& obj, D res)
{
    using elem_type = typename C::value_type;
    std::for_each(obj.cbegin(), obj.cend(), [&res](const elem_type& elem){ REQUIRE(elem == res); });
}