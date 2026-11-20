#!/usr/bin/env python3
"""Extract pinned output-predictor steps for source-compositional verification.

The production methods are unchanged. Each generated function binds their
original members and locals explicitly. All source contracts, loop induction,
and reviewed correspondence are required; this is not a C++ refinement prover.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

REPO_DIR = Path(__file__).resolve().parents[3]
PINS = {
    "integrations/px4/module/OutputPredictor.hpp": "484a7256490a545f22ea4eab133991f47fa39e149a6a5b72844c9614c22839d2",
    "include/formal_eskf/runtime/ring_buffer.hpp": "4e14776bcf6f1d4ba499c93a289a9ec6ceab1dc2b24173c7920f988bba9a0ac5",
    "include/formal_eskf/linalg/matrix.hpp": "09ad7208f3d44d6686a41eb8eabc3e4963562f3f5c2d175493db1a38c402cddc",
    "formal/cpp/esbmc/support.hpp": "067c556f362de8f5ad21e8672b8711ce00311fe85f9898c8091e8ca6dbc8962e",
    "formal/cpp/esbmc/contracts.hpp": "6c5ba84e914ccb08b98acf02b204bbb07e455b166ad754f2634e6c3c65f0180c",
    "formal/cpp/esbmc/imu_owner_support.hpp": "7e33850d8ffa7c24f2cd182995804dabbd3def5cbf0429d9174a833b183f6d18",
    "integrations/px4/module/DeploymentProfile.hpp": "d665bc65cf9c6e8f2c3d1af0642ef8852e5e0d39455b047ef4b2e2165c030664",
    "integrations/px4/module/DeploymentConfig.hpp": "015b515863ec85cbc1ebd4994ea6c2b6a1a8191fea825a63f48fc7a72935a8dd",
    "formal/cpp/esbmc/output/support.hpp": "b0cb7a6ebad3729f4f49396d2ba4c7a753a89da65b6a739061c8bbbf16f4e57e",
    "formal/cpp/esbmc/output/frames.hpp": "774787b8c1f76730fd1b11531be998763e6eac815f5db022ca2b681c05cdeef5",
    "formal/cpp/esbmc/output/boundary.hpp": "6b57e88d2a17acd5ce3b7048b3f21ed3461b9e6d4e096dd6083e20c39dd4fd76",
    "formal/cpp/esbmc/output/selected_frame.hpp": "dacd7d31b93ee2b39eeeb1dd01d3f991cd7729a8818d8e25b9cce823ed0a24ec",
}
HEADER = '''// Verification-only extracted source; production methods are unchanged.
#pragma once
#include "output/support.hpp"
namespace output_steps
{
using namespace output_proof;
namespace linalg = formal_eskf::linalg;
using formal_eskf::succeeded;
'''
FOOTER = '} // namespace output_steps\n'
ALIASES = {'value_type': 'Scalar', 'vector3_type': 'Vector', 'quaternion_type': 'Quaternion',
           'TrackingError': 'Output::TrackingError', 'Fault': 'Output::Fault'}
MEMBERS = ('m_history', 'm_staged', 'm_current', 'm_fault', 'm_fusion_time_us', 'm_filter_interval_s',
           'm_accel_bias', 'm_gyro_bias', 'm_angular_rate_correction', 'm_velocity_error_sum',
           'm_position_error_sum', 'm_error', 'm_tracking')


def sha256(DATA):
    return hashlib.sha256(DATA).hexdigest()


def check_sources():
    for NAME, EXPECTED in PINS.items():
        if sha256((REPO_DIR / NAME).read_bytes()) != EXPECTED:
            raise ValueError(f'source correspondence requires review: {NAME}')


def body_range(TEXT, MARKER):
    # Deliberately limited to the pinned source, not a general C++ parser.
    if TEXT.count(MARKER) != 1:
        raise ValueError(f'ambiguous source marker: {MARKER}')
    START = TEXT.index('{', TEXT.index(MARKER)) + 1
    END, DEPTH = START, 1
    while DEPTH:
        DEPTH += (TEXT[END] == '{') - (TEXT[END] == '}')
        END += 1
    return START, END - 1


def body(TEXT, MARKER):
    START, END = body_range(TEXT, MARKER)
    return TEXT[START:END]


def replace_once(TEXT, BEFORE, AFTER):
    if TEXT.count(BEFORE) != 1:
        raise ValueError(f'unexpected source statement: {BEFORE}')
    return TEXT.replace(BEFORE, AFTER)


def member_bindings(TEXT):
    return ''.join(f'    auto & {NAME} = owner.{NAME};\n' for NAME in MEMBERS
                   if re.search(r'\b' + NAME + r'\b', TEXT))


def function(SIGNATURE, BODY, LOCAL_BINDINGS=''):
    TYPES = ''.join(f'    using {NAME} = {TYPE};\n' for NAME, TYPE in ALIASES.items()
                    if re.search(r'\b' + NAME + r'\b', BODY))
    return SIGNATURE + '\n{\n' + TYPES + member_bindings(BODY) + LOCAL_BINDINGS + BODY + '\n}\n\n'


def generate_history(TEXT):
    TRANSLATION = body(TEXT, 'bool OutputPredictor<Linalg>::stage_translation(')
    COMMIT = body(TEXT, 'void OutputPredictor<Linalg>::commit_history()')
    RESET = body(TEXT, 'bool OutputPredictor<Linalg>::apply_reset(')
    LOOP = 'for (std::size_t record = 0; record < count; ++record)'
    STAGE = body(TRANSLATION, 'for (std::size_t count = 0; count < m_history.entries(); ++count)')
    COPY = body(COMMIT, 'for (std::size_t count = 0; count < m_history.entries(); ++count)')
    START, END = body_range(RESET, LOOP)
    TRANSFORM = RESET[START:END]
    PREFIX = RESET[:RESET.index(LOOP)]
    SUFFIX = RESET[END + 1:]
    if STAGE.count('return false;') != 1 or 'return' in COPY:
        raise ValueError('unexpected staging/commit control flow')
    OUTPUT = function('inline bool translation(Output & owner, std::size_t & index,\n'
                      '                        Vector const & velocity_delta, Vector const & position_delta)',
                      STAGE + '\n    return true;')
    OUTPUT += function('inline void commit(Output & owner, std::size_t & index)', COPY)
    OUTPUT += function('inline void finish(Output & owner)', COMMIT[COMMIT.index('    m_current ='):])
    OUTPUT += function('inline bool transform(Output & owner, std::size_t & index, bool seed_only,\n'
                       '                      Quaternion const & q_delta, Vector const & velocity_delta_n,\n'
                       '                      Vector const & position_delta_n)',
                       replace_once(TRANSFORM, 'return fail(Fault::numerical_failure);',
                                    'return owner.fail(Fault::numerical_failure);') + '\n    return true;')
    OUTPUT += '''struct ResetPlan
{
    bool seed_only;
    std::size_t count;
    std::size_t index;
};

'''
    PLAN = replace_once(PREFIX, 'valid_quaternion(q_delta)', 'owner.valid_quaternion(q_delta)')
    PLAN = replace_once(PLAN, 'return fail(Fault::invalid_state);', 'return owner.fail(Fault::invalid_state);')
    OUTPUT += function('inline bool reset_plan(Output & owner, Quaternion const & q_delta,\n'
                       '                       Vector const & velocity_delta_n, Vector const & position_delta_n,\n'
                       '                       ResetPlan & plan)',
                       PLAN + '\n    plan = {seed_only, count, index};\n    return true;')
    OUTPUT += function('inline bool reset_finish(Output & owner, bool seed_only)',
                       replace_once(SUFFIX, 'commit_history();', 'owner.commit_history();'))
    SEGMENTS = dict(translation=TRANSLATION, commit=COMMIT, reset=RESET, translation_step=STAGE,
                    commit_step=COPY, reset_prefix=PREFIX, reset_loop=RESET[RESET.index(LOOP):END + 1],
                    reset_suffix=SUFFIX)
    if PREFIX + SEGMENTS['reset_loop'] + SUFFIX != RESET:
        raise ValueError('reset decomposition does not cover the original body')
    return HEADER + OUTPUT + FOOTER, SEGMENTS


def generate_feedback(TEXT):
    BODY = body(TEXT, 'bool OutputPredictor<Linalg>::correct(')
    NORMALIZE = BODY.index('    quaternion_type q_error;')
    NUMERICS = BODY.index('    value_type const sign_scale')
    FINISH = BODY.index('    if (!linalg::all_finite(angular_rate_correction)')
    PARTS = dict(plan=BODY[:NORMALIZE], normalize=BODY[NORMALIZE:NUMERICS],
                 numerics=BODY[NUMERICS:FINISH], finish=BODY[FINISH:])
    if ''.join(PARTS.values()) != BODY:
        raise ValueError('feedback decomposition does not cover the original body')
    OUTPUT = '''struct FeedbackValues
{
    Output::TrackingError error;
    Scalar interval_s;
    Vector angular_rate_correction;
    Vector velocity_sum;
    Vector position_sum;
    Vector velocity_delta;
    Vector position_delta;
};

'''
    PLAN = replace_once(PARTS['plan'], 'auto const * sample = delayed();', 'sample = owner.delayed();')
    PLAN = replace_once(PLAN, 'valid_state(state)', 'owner.valid_state(state)')
    PLAN = PLAN.replace('return fail(', 'return owner.fail(')
    OUTPUT += function('inline bool correction_plan(Output & owner, std::uint64_t time_us,\n'
                       '                            State const & state, Sample const * & sample)',
                       PLAN + '    return true;')
    NORMALIZATION = replace_once(PARTS['normalize'], '    quaternion_type q_error;\n', '')
    NORMALIZATION = replace_once(NORMALIZATION, 'return fail(', 'return owner.fail(')
    OUTPUT += function('inline bool correction_normalize(Output & owner, State const & state,\n'
                       '                                 Sample const * sample, Quaternion & q_error)',
                       NORMALIZATION + '    return true;')
    OUTPUT += function('inline void correction_numerics(Output & owner, State const & state, Sample const * sample,\n'
                       '                                Quaternion const & q_error, std::uint64_t time_us,\n'
                       '                                FeedbackValues & result)',
                       PARTS['numerics'] + '    result = {error, interval_s, angular_rate_correction, velocity_sum,\n'
                       '              position_sum, velocity_delta, position_delta};')
    FINISH = replace_once(PARTS['finish'], 'stage_translation(', 'owner.stage_translation(')
    FINISH = replace_once(FINISH, 'commit_history();', 'owner.commit_history();')
    FINISH = replace_once(FINISH, 'return fail(', 'return owner.fail(')
    VALUES = ('error', 'interval_s', 'angular_rate_correction', 'velocity_sum', 'position_sum',
              'velocity_delta', 'position_delta')
    BINDINGS = ''.join(f'    auto const & {NAME} = values.{NAME};\n' for NAME in VALUES)
    OUTPUT += function('inline bool correction_finish(Output & owner, State const & state,\n'
                       '                              std::uint64_t time_us, FeedbackValues const & values)',
                       FINISH, BINDINGS)
    INTERVAL_START = PARTS['numerics'].index('    auto const interval_s =')
    INTERVAL_END = PARTS['numerics'].index(';', INTERVAL_START) + 1
    INTERVAL = PARTS['numerics'][INTERVAL_START:INTERVAL_END]
    INTERVAL_OUTPUT = '''inline Scalar interval_average(Scalar m_filter_interval_s, Sample const * sample)
{
    using value_type = Scalar;
''' + INTERVAL + '\n    return interval_s;\n}\n'
    PARTS.update(correct=BODY, interval=INTERVAL)
    return HEADER + OUTPUT + FOOTER, HEADER + INTERVAL_OUTPUT + FOOTER, PARTS


def generate(OUTPUT_DIR):
    check_sources()
    for DIRECTORY in ('include', 'integrations', 'formal', 'tests', 'scripts', 'configs'):
        if OUTPUT_DIR.resolve().is_relative_to((REPO_DIR / DIRECTORY).resolve()):
            raise ValueError('verification output must not replace production or proof sources')
    TEXT = (REPO_DIR / 'integrations/px4/module/OutputPredictor.hpp').read_text()
    HISTORY, HISTORY_PARTS = generate_history(TEXT)
    FEEDBACK, INTERVAL, FEEDBACK_PARTS = generate_feedback(TEXT)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUTS = dict(source_steps=HISTORY, feedback_steps=FEEDBACK, interval_step=INTERVAL)
    for NAME, CONTENT in OUTPUTS.items():
        (OUTPUT_DIR / (NAME + '.hpp')).write_text(CONTENT)
    MANIFEST = {
        'claim': 'Source contracts plus separate loop/temporal induction and reviewed correspondence; not automatic C++ refinement.',
        'source_sha256': PINS,
        'generator_sha256': sha256(Path(__file__).read_bytes()),
        'history_fragments': {NAME: sha256(BODY.encode()) for NAME, BODY in HISTORY_PARTS.items()},
        'feedback_fragments': {NAME: sha256(BODY.encode()) for NAME, BODY in FEEDBACK_PARTS.items()},
        'generated_sha256': {NAME + '.hpp': sha256(CONTENT.encode()) for NAME, CONTENT in OUTPUTS.items()},
        'transformation': 'Bind original members by reference and member calls by explicit receiver; pass boundary locals as proof-only parameters; step fallthrough returns true. Original loops, guards and publication order require correspondence review. No numerical or branch expressions are replaced.',
    }
    (OUTPUT_DIR / 'output-steps.json').write_text(json.dumps(MANIFEST, indent=2) + '\n')


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument('output_dir', type=Path, help='Verification-only include directory.')
    ARGS = PARSER.parse_args()
    try:
        generate(ARGS.output_dir)
    except (OSError, ValueError, IndexError) as ERROR:
        print(f'error: {ERROR}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
