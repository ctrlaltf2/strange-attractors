#pragma once
#ifdef USE_GMP
#include <boost/multiprecision/gmp.hpp>

using namespace boost::multiprecision;

using bigfloat = mpf_float;
#elif USE_DEC_FLOAT
#include <boost/multiprecision/cpp_dec_float.hpp>

using namespace boost::multiprecision;

using bigfloat = cpp_dec_float_100;
#else
#include <cmath>

using std::sin;
using std::cos;

using bigfloat = double;

#endif
