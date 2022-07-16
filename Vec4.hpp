#pragma once
#include <boost/multiprecision/gmp.hpp>

using namespace boost::multiprecision;

struct Vec4 {
    mpf_float x, y, z, w;
};
