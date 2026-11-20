// SPDX-License-Identifier: BSD-3-Clause

#include "height_state_support.hpp"

// Qualify the explicit-reference lowering against the actual Sample payload.
// The original conditional lvalue currently fails the frontend diagnostic; its
// native behavior and reviewed C++ correspondence remain separate evidence.
void verify_height_selection(height_proof::Sample barometer, height_proof::Sample gnss, bool select_barometer,
                             bool shared_sample)
{
    using namespace height_proof;
    auto const old_barometer = barometer;
    auto const old_gnss = gnss;
    Sample const * gnss_input = &gnss;
    if (shared_sample)
    {
        gnss_input = &barometer;
    }
    Sample const * expected = &barometer;
    Source source = Source::barometer;
    if (!select_barometer)
    {
        expected = gnss_input;
        source = Source::gnss;
    }
    auto const & selected = Manager::selected_sample(source, barometer, *gnss_input);
    __ESBMC_assert(&selected == expected && same_sample(selected, *expected),
                   "Height selection: exact borrowed object identity and every payload field");
    __ESBMC_assert(same_sample(barometer, old_barometer) && same_sample(gnss, old_gnss),
                   "Height selection: neither source payload is modified");
}

int main() { return 0; }
