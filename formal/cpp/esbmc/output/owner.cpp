// SPDX-License-Identifier: BSD-3-Clause

#include "support.hpp"
using namespace output_proof;

void verify_output_initialization()
{
    Output owner;
    empty_owner(owner);
    __ESBMC_assert(owner.m_history.m_length == 0 &&
                       same_configuration(owner.m_configuration, formal_eskf::px4::TimingConfiguration{}),
                   "Output constructor retains unconfigured history and default timing");
}

void verify_output_session_reset(Output owner)
{
    auto const length = owner.m_history.m_length;
    auto const configuration = owner.m_configuration;
    owner.reset();
    empty_owner(owner);
    __ESBMC_assert(owner.m_history.m_length == length && same_configuration(owner.m_configuration, configuration),
                   "Output session reset preserves history length and complete configuration");
}
