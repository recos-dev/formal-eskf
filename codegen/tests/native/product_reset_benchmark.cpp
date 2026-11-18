/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define PRODUCT_KERNELS_ONLY
#include "products.cpp"
#include <algorithm>

template <typename S> void benchmark()
{
    for (unsigned seed = 0U; seed < 16U; ++seed)
    {
        Random random{28374U + seed};
        Matrix<S, 15U, 15U> P;
        positive(P, random);
        Matrix<S, 3U, 1U> angle;
        fill(angle, random);
        Matrix<S, 3U, 3U> J;
        if (!formal_eskf::succeeded(formal_eskf::so3::right_jacobian(angle, J)))
        {
            std::abort();
        }
        auto dense = dense_ins_reset(P, J);
        auto direct = block_ins_reset(P, J);
        for (std::size_t i = 0U; i < 15U; ++i)
        {
            for (std::size_t j = 0U; j < 15U; ++j)
            {
                auto scale = std::max(S{1}, std::abs(dense(i, j)));
                if (std::abs(dense(i, j) - direct(i, j)) > S{64} * std::numeric_limits<S>::epsilon() * scale)
                {
                    std::abort();
                }
            }
        }
        for (unsigned round = 0U; round < 9U; ++round)
        {
            for (unsigned order = 0U; order < 3U; ++order)
            {
                auto variant = (round + order) % 3U;
                auto start = std::chrono::steady_clock::now();
                for (unsigned count = 0U; count < 4096U; ++count)
                {
                    asm volatile("" ::: "memory");
                    if (variant == 0U)
                    {
                        dense = dense_ins_reset(P, J);
                    }
                    if (variant == 1U)
                    {
                        dense = generated_ins_reset(P, J);
                    }
                    if (variant == 2U)
                    {
                        dense = block_ins_reset(P, J);
                    }
                    // Keep the complete returned matrix observable to the compiler.
                    asm volatile("" : : "m"(dense) : "memory");
                    checksum = static_cast<double>(dense(0U, 0U));
                }
                double ns =
                    std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - start).count() / 4096U;
                std::printf("{\"bits\":%zu,\"seed\":%u,\"round\":%u,\"variant\":%u,\"ns\":%.9g}\n", sizeof(S) * 8U,
                            seed, round, variant, ns);
            }
        }
    }
}

int main()
{
#ifndef USE_PX4
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    benchmark<float>();
    benchmark<double>();
}
