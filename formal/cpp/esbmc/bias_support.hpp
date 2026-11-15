/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "correction_support.hpp"

#ifndef FORMAL_ESKF_PROOF_BIAS_CONTROL
#define FORMAL_ESKF_PROOF_BIAS_CONTROL 0
#endif
static_assert(FORMAL_ESKF_PROOF_BIAS_CONTROL == 0 || FORMAL_ESKF_PROOF_BIAS_CONTROL == 1);

namespace bias_proof
{
using permissions_type = formal_eskf::configuration::Ins::BiasUpdate;

// Independent row-selection oracle: one Boolean for each of the six bias rows.
inline bool permitted(permissions_type const & permissions, std::size_t row)
{
    return row < 9U ? true : (row < 12U ? permissions.accelerometer[row - 9U] : permissions.gyroscope[row - 12U]);
}

inline bool same_permissions(permissions_type const & a, permissions_type const & b)
{
    bool equal = true;
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        equal = a.accelerometer[i] == b.accelerometer[i] && a.gyroscope[i] == b.gyroscope[i] && equal;
    }
    return equal;
}

inline bool constrained_gain(correction_proof::Gain const & actual, correction_proof::Gain const & original,
                             permissions_type const & permissions)
{
    bool equal = true;
    for (std::size_t row = 0U; row < correction_proof::state_size; ++row)
    {
        for (std::size_t column = 0U; column < correction_proof::measurement_size; ++column)
        {
            auto const expected = permitted(permissions, row) ? original(row, column) : correction_proof::Scalar{0};
            equal = correction_proof::same(actual(row, column), expected) && equal;
        }
    }
    return equal;
}
} /* end namespace bias_proof */
