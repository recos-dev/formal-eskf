/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/runtime/accelerometer_sample.hpp>

namespace formal_eskf::example
{

template <typename Linalg, std::uint64_t IntervalUs>
using ReplayAccelerometer = runtime::AccelerometerSample<Linalg, IntervalUs>;

} /* end namespace formal_eskf::example */
