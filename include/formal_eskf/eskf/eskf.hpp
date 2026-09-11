/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/**
 * @file
 * Master header for the compile-time configurable ESKF core.
 */

#include <formal_eskf/eskf/configuration/ahrs.hpp>
#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/eskf/prediction.hpp>
#include <formal_eskf/eskf/process_noise.hpp>
#include <formal_eskf/eskf/covariance_prediction.hpp>
#include <formal_eskf/eskf/injection.hpp>
#include <formal_eskf/eskf/correction.hpp>
#include <formal_eskf/eskf/position.hpp>
#include <formal_eskf/eskf/velocity.hpp>
#include <formal_eskf/eskf/magnetometer.hpp>
