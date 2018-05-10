//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <type_traits>

#pragma once

/// \brief Convenience alias for a constant
/// \tparam I		The constant.
template < auto I >
using constant = std::integral_constant< decltype(I), I >;
