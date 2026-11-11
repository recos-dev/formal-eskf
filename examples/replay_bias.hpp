/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/runtime/imu_bias_inhibition.hpp>

namespace formal_eskf::example
{

struct ReplayImuFlags
{
    int at_rest = -1;
    int accelerometer_clipping = -1;
};

template <typename Linalg> using ReplayBiasLearning = runtime::ImuBiasInhibition<Linalg>;

} /* end namespace formal_eskf::example */
