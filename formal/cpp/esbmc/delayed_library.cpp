// SPDX-License-Identifier: BSD-3-Clause

#include "contracts.hpp"
#include "delayed_library.hpp"
#include <cstdint>

void verify_delayed_max(std::uint64_t first, std::uint64_t second, std::uint64_t third)
{
    auto const result = std::formal_eskf_proof_max({first, second, third});
    __ESBMC_assert(result >= first && result >= second && result >= third &&
                       (result == first || result == second || result == third),
                   "delayed library: exact unsigned maximum of all three complete-width inputs");
}

int main() { return 0; }
