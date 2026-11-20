#!/usr/bin/env python3
"""Extract pinned module callers for source-compositional verification.

Complete method bodies retain their statements, with the owning class renamed.
Explicitly listed source steps retain selected statements at reviewed frames.
Proof bindings replace callees at reviewed contracts; they do not model PX4
scheduling, target ABI, host diagnostics, or opaque numerical payloads.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

REPO_DIR = Path(__file__).resolve().parents[3]
PINS = {
    "integrations/px4/module/FormalEskf.cpp": "241f793ab63fcabb6542ae708540ff6940be6b551aa5974e52de53857b88a147",
    "integrations/px4/module/FormalEskf.hpp": "ffe1420cb9a9f8e5ae2f440300342add58c40c96e51afcec204eb503fbf1b843",
    "integrations/px4/module/FormalEskfOutput.cpp": "6f3d790874661887e1e388063c8fd8969a696f2b17afe1a5650d88fd020ac760",
    "integrations/px4/module/DeploymentProfile.hpp": "d665bc65cf9c6e8f2c3d1af0642ef8852e5e0d39455b047ef4b2e2165c030664",
    "integrations/px4/module/ImuPrediction.hpp": "37adf0033cf10196abd2ae3ae570b5496cb4248585e811b40a86103e5e318eb6",
    "integrations/px4/module/FormalEskfNavigation.cpp": "3c7479b6cf3f026a71f98eff34730278c8b61f097947f803428e83d69c27ba32",
    "integrations/px4/module/FormalEskfAttitude.cpp": "f3df7057b135a74fe3a045577e35fb0faaf80453ca724989b0d5c032a315bba9",
    "integrations/px4/module/ImuHistory.hpp": "1b0f750bd57e01f82ba9a2e0d20969a6558b01b64d05eb44d16d3804d8287fb8",
    "integrations/px4/module/DeploymentConfig.hpp": "015b515863ec85cbc1ebd4994ea6c2b6a1a8191fea825a63f48fc7a72935a8dd",
    "examples/replay_ins.cpp": "c8f6cb10460abee6f3e14c77dd02ba57c82d8d05fc1574258299b1844981328a",
}
GROUPS = {
    "callback": ("FormalEskf.cpp", ("void FormalEskf::Run()",)),
    "feedback": ("FormalEskf.cpp", ("bool FormalEskf::check_timing(bool succeeded)",
                                      "void FormalEskf::advance_fusion()")),
    "prediction": ("FormalEskf.cpp", ("bool FormalEskf::check(status_type status)",
                                        "bool FormalEskf::check_timing(bool succeeded)",
                                        "void FormalEskf::predict_and_fuse(")),
    "process": ("FormalEskf.cpp", ("bool FormalEskf::check_timing(bool succeeded)",
                                     "bool FormalEskf::accept_imu(", "void FormalEskf::process_imu(")),
    "initialization": ("FormalEskf.cpp", ("bool FormalEskf::check_timing(bool succeeded)",
                                            "void FormalEskf::initialize(")),
    "validity": ("FormalEskfOutput.cpp", ("FormalEskf::OutputValidity FormalEskf::output_validity(",)),
    "selection": ("FormalEskf.cpp", ("void FormalEskf::select_observations()",)),
    "context": ("FormalEskf.cpp", ("FormalEskf::observation_history_type::Context FormalEskf::startup_context(",)),
    "status_check": ("FormalEskf.cpp", ("bool FormalEskf::check(status_type status)",)),
    "publication": ("FormalEskfOutput.cpp", ("void FormalEskf::publish(imu_type const & imu)",)),
    "reset": ("FormalEskf.cpp", ("bool FormalEskf::check_timing(bool succeeded)",
                                   "bool FormalEskf::reset_output(")),
    "recent": ("FormalEskf.cpp", ("bool FormalEskf::recent(",)),
    "gnss": ("FormalEskfNavigation.cpp", ("void FormalEskf::Gnss::change_source(",
                                           "void FormalEskf::update_gps_quality()")),
    "magnetic_reset": ("FormalEskfAttitude.cpp", ("FormalEskf::status_type FormalEskf::reset_magnetic_heading(",)),
    "height": ("FormalEskfNavigation.cpp", ("void FormalEskf::fuse_height()",)),
    "startup": ("FormalEskf.cpp", ("bool FormalEskf::load_timing_parameters()",
                                     "bool FormalEskf::load_startup_parameters()", "bool FormalEskf::init()")),
}
PUBLICATIONS = (
    ("void FormalEskf::publish_attitude(", "attitude", "vehicle_attitude_s"),
    ("vehicle_local_position_s FormalEskf::publish_local_position(", "local", "vehicle_local_position_s"),
    ("void FormalEskf::publish_global_position(", "global", "vehicle_global_position_s"),
    ("void FormalEskf::publish_odometry(", "odometry", "vehicle_odometry_s"),
    ("estimator_status_s FormalEskf::publish_estimator_status(", "status", "estimator_status_s"),
    ("estimator_status_flags_s FormalEskf::status_flags(", "flags", "estimator_status_flags_s"),
    ("void FormalEskf::publish_sensor_bias(", "bias", "estimator_sensor_bias_s"),
)
RESET_BRANCHES = (
    ("velocity", "bool FormalEskf::fuse_gps_velocity(",
     "if (m_gnss.lifecycle.state().velocity.action == GnssFusion::Action::reset)",
     "Vector const & velocity, Vector const & velocity_variances"),
    ("vertical_velocity", "bool FormalEskf::fuse_gps_velocity(",
     "if (m_height_aiding.reset_required(m_time_us, {health.last_update_time_us, health.last_good_time_us}))",
     "Vector const & velocity, Vector const & velocity_variances"),
    ("position", "bool FormalEskf::fuse_gps_position()",
     "if (m_gnss.lifecycle.state().horizontal_position.action == GnssFusion::Action::reset)",
     "Vector2 const & xy, double position_variance"),
)


def sha256(DATA):
    return hashlib.sha256(DATA).hexdigest()


def method(TEXT, SIGNATURE):
    # A deliberately limited extractor for these hash-pinned, brace-balanced
    # methods. Unexpected source needs review; this is not a C++ parser.
    if TEXT.count(SIGNATURE) != 1:
        raise ValueError(f"ambiguous method: {SIGNATURE}")
    START = TEXT.index(SIGNATURE)
    END = TEXT.index("{", START) + 1
    DEPTH = 1
    while DEPTH:
        DEPTH += (TEXT[END] == "{") - (TEXT[END] == "}")
        END += 1
    return TEXT[START:END]


def branch_body(TEXT, CONDITION):
    if TEXT.count(CONDITION) != 1:
        raise ValueError(f"ambiguous reset branch: {CONDITION}")
    START = TEXT.index("{", TEXT.index(CONDITION) + len(CONDITION)) + 1
    END, DEPTH = START, 1
    while DEPTH:
        DEPTH += (TEXT[END] == "{") - (TEXT[END] == "}")
        END += 1
    return TEXT[START:END - 1]


def generate(OUTPUT_DIR):
    for DIRECTORY in ("include", "integrations", "formal", "tests", "scripts", "configs"):
        if OUTPUT_DIR.resolve().is_relative_to((REPO_DIR / DIRECTORY).resolve()):
            raise ValueError("verification output must not replace source files")
    SOURCES = {}
    for NAME, EXPECTED in PINS.items():
        DATA = (REPO_DIR / NAME).read_bytes()
        if sha256(DATA) != EXPECTED:
            raise ValueError(f"source correspondence requires review: {NAME}")
        SOURCES[Path(NAME).name] = DATA.decode()
    OUTPUTS = {}
    MANIFEST = {"source_sha256": PINS, "generator_sha256": sha256(Path(__file__).read_bytes()),
                "transformation": "Complete methods rename FormalEskf to Flow. Selected source steps and host exception projections are listed separately.",
                "methods": {}, "publication_prefixes": {}, "reset_branches": {}, "source_steps": {},
                "generated_sha256": {}}
    CONSTANTS = []
    for NAME in ("backup_interval_us", "status_interval_us", "seconds_per_microsecond"):
        MATCHES = re.findall(r"^inline constexpr [^\n]*\b" + NAME + r" = [^\n]*$",
                             SOURCES["DeploymentProfile.hpp"], re.MULTILINE)
        if len(MATCHES) != 1:
            raise ValueError(f"ambiguous profile constant: {NAME}")
        CONSTANTS.append(MATCHES[0])
    TEXT = "namespace profile\n{\n" + "\n".join(CONSTANTS) + "\n}\n"
    CONSTANT = re.findall(r"^inline constexpr [^\n]*\brest_maximum_age_us = [^\n]*$",
                          SOURCES["DeploymentProfile.hpp"], re.MULTILINE)
    if len(CONSTANT) != 1:
        raise ValueError("ambiguous IMU context age")
    TEXT += "namespace profile::imu\n{\n" + CONSTANT[0] + "\n}\n"
    for NAMESPACE, NAMES in {
            "output": ("maximum_age_us", "navigation_interval_us", "dead_reckoning_age_us"),
            "history": ("minimum_period_us", "maximum_period_us"),
            "gnss": ("context_maximum_age_us", "maximum_sample_age_us"),
            "magnetic": ("context_maximum_age_us",)}.items():
        BLOCK = method(SOURCES["DeploymentProfile.hpp"], f"namespace {NAMESPACE}\n{{")
        TEXT += f"namespace profile::{NAMESPACE}\n{{\n"
        for NAME in NAMES:
            MATCHES = re.findall(r"^inline constexpr [^\n]*\b" + NAME + r" = [^\n]*$", BLOCK, re.MULTILINE)
            if len(MATCHES) != 1:
                raise ValueError(f"ambiguous constant: {NAMESPACE}::{NAME}")
            TEXT += MATCHES[0] + "\n"
        TEXT += "}\n"
    OUTPUTS["module_constants.inc"] = TEXT
    TEXT = "namespace formal_eskf::px4\n{\n" + method(
        SOURCES["ImuPrediction.hpp"], "inline bool imu_clipping_acceptable(") + "\n}\n"
    OUTPUTS["module_clipping.inc"] = TEXT
    TEXT = "namespace formal_eskf::px4\n{\n" + method(
        SOURCES["ImuHistory.hpp"], "struct TimingParameters\n{") + ";\n}\n"
    OUTPUTS["module_timing_type.inc"] = TEXT
    for GROUP, (FILE, SIGNATURES) in GROUPS.items():
        METHODS = [method(SOURCES[FILE], SIGNATURE) for SIGNATURE in SIGNATURES]
        for SIGNATURE, TEXT in zip(SIGNATURES, METHODS):
            MANIFEST["methods"][SIGNATURE] = {"file": FILE, "sha256": sha256(TEXT.encode())}
        TEXT = "// Verification-only source extraction; do not edit.\n"
        TEXT += "\n\n".join(METHOD.replace("FormalEskf::", "Flow::") for METHOD in METHODS) + "\n"
        NAME = f"module_{GROUP}.inc"
        OUTPUTS[NAME] = TEXT
    TEXT = "// Exact message-initialization prefixes; payload serialization is separate.\n"
    for SIGNATURE, NAME, TYPE in PUBLICATIONS:
        METHOD = method(SOURCES["FormalEskfOutput.cpp"], SIGNATURE)
        BODY = METHOD[METHOD.index("{") + 1:-1]
        LINES = BODY.splitlines()[1:4]
        PREFIX = "\n".join(LINES)
        if len(LINES) != 3 or LINES[0].strip() != f"{TYPE} {NAME}{{}};":
            raise ValueError(f"unexpected message initialization: {NAME}")
        # These reviewed methods neither alias nor subsequently assign either
        # timestamp. The native fixture covers the full serializer/uORB binding.
        for FIELD in ("timestamp", "timestamp_sample"):
            if len(re.findall(r"\b" + NAME + r"\." + FIELD + r"\b", BODY)) != 1:
                raise ValueError(f"timestamp use requires renewed frame review: {NAME}.{FIELD}")
        MANIFEST["publication_prefixes"][NAME] = {
            "method": SIGNATURE, "method_sha256": sha256(METHOD.encode()),
            "prefix_sha256": sha256(PREFIX.encode()),
            "boundary": "Original initialization statements; append a return for their values. Remaining payload writes and publish calls need the reviewed no-timestamp-write frame and native binding evidence.",
        }
        TEXT += f"Message Flow::{NAME}_epochs(std::uint64_t now) const\n{{\n{PREFIX}\n    return {NAME};\n}}\n"
    OUTPUTS["module_epochs.inc"] = TEXT
    TEXT = "// Exact selected reset branches; the wrapper supplies the enclosing locals.\n"
    for NAME, SIGNATURE, CONDITION, PARAMETERS in RESET_BRANCHES:
        METHOD = method(SOURCES["FormalEskfNavigation.cpp"], SIGNATURE)
        BODY = branch_body(METHOD, CONDITION)
        TEXT += f"bool Flow::{NAME}_reset_path({PARAMETERS})\n{{\n"
        TEXT += "    using namespace formal_eskf::runtime;\n    using Group = gnss_lifecycle_type::Group;\n"
        TEXT += BODY + "\n    return true;\n}\n"
        MANIFEST["reset_branches"][NAME] = {
            "method": SIGNATURE, "method_sha256": sha256(METHOD.encode()),
            "condition": CONDITION, "body_sha256": sha256(BODY.encode()),
            "boundary": "The original selected-branch statements and early returns; enclosing locals become typed arguments, and normal branch fallthrough returns true. Selection/preceding fusion correctness is a separate caller contract.",
        }
    OUTPUTS["module_reset_paths.inc"] = TEXT
    METHOD = method(SOURCES["FormalEskfAttitude.cpp"], "void FormalEskf::fuse_mag()")
    MARKER = "    bool reset = false;"
    if METHOD.count(MARKER) != 1 or METHOD.count("reset_magnetic_heading(") != 1:
        raise ValueError("magnetic reset suffix requires renewed decomposition review")
    SUFFIX = METHOD[METHOD.index(MARKER):-1]
    TEXT = "void Flow::finish_magnetic(bool fused, bool rejected, LifecycleState const & previous,\n"
    TEXT += "                           Conditions const & conditions, Event const & event)\n{\n"
    TEXT += "    auto const & lifecycle = m_magnetic.lifecycle.state();\n" + SUFFIX + "}\n"
    OUTPUTS["module_magnetic_finish.inc"] = TEXT
    MANIFEST["source_steps"]["magnetic_finish"] = {
        "method_sha256": sha256(METHOD.encode()), "suffix_sha256": sha256(SUFFIX.encode()),
        "boundary": "Exact reset/feedback/reporting suffix, entered after successful preceding observation work. Enclosing locals are explicit parameters; lifecycle remains a reference to the same mutable owner.",
    }
    replay_steps(SOURCES["replay_ins.cpp"], OUTPUTS, MANIFEST)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for NAME, TEXT in OUTPUTS.items():
        (OUTPUT_DIR / NAME).write_text(TEXT)
        MANIFEST["generated_sha256"][NAME] = sha256(TEXT.encode())
    (OUTPUT_DIR / "manifest.json").write_text(json.dumps(MANIFEST, indent=2) + "\n")


def replay_steps(SOURCE, OUTPUTS, MANIFEST):
    def between(TEXT, FIRST, LAST):
        if TEXT.count(FIRST) != 1:
            raise ValueError(f"ambiguous replay step: {FIRST}")
        START = TEXT.index(FIRST)
        return TEXT[START:TEXT.index(LAST, START)]

    def save(NAME, TEXT, BOUNDARY):
        # Preserve exception transfer and prior writes, excluding diagnostic
        # string allocation/formatting from this source-control claim.
        PROJECTED, COUNT = re.subn(r'throw std::runtime_error\("[^"\n]*"\);',
                                  'throw ReplayFailure{};', TEXT)
        OUTPUTS[f"replay_{NAME}.inc"] = PROJECTED
        MANIFEST["source_steps"][f"replay_{NAME}"] = {
            "file": "examples/replay_ins.cpp", "source_sha256": sha256(TEXT.encode()),
            "exception_diagnostics_projected": COUNT, "boundary": BOUNDARY,
        }

    IMU = between(SOURCE, "        auto const next_time = timestamp(imu_row[0U]);",
                  "        check_state(state, covariance);")
    save("imu", IMU, "Exact tick prefix through prediction count. Decoded timestamp/IMU ingress and numeric/policy callees have typed contracts; exceptions terminate this tick without rolling back earlier successful prediction.")
    MAG = method(SOURCE, "while (has_magnetometer && timestamp(magnetic_row[0U]) <= time)")
    save("magnetic", MAG.replace("while (", "if (", 1), "One arbitrary iteration with the original guard/body (while becomes if for the induction step). Stream advance, payload arithmetic and context parsing are callee contracts. The full parent loop contains no fusion call.")
    MANIFEST["source_steps"]["replay_magnetic"]["parent_sha256"] = sha256(MAG.encode())
    NAV = method(SOURCE, "while (has_navigation && timestamp(navigation_row[0U]) <= time)")
    GUARD = between(NAV, "            auto const source_time = timestamp(navigation_row[0U]);",
                    "            auto const model_sample =")
    CURSOR = between(NAV, "            updates << source_time", "            ++navigation_attempts;")
    ADVANCE = "            has_navigation = read_row(navigation, navigation_row, quality_parameters.has_value());\n"
    if NAV.count(ADVANCE.strip()) != 1 or NAV.count("last_navigation_time =") != 1 or NAV.count("read_row(") != 2:
        raise ValueError("replay navigation cursor frame requires renewed review")
    save("navigation_guard", GUARD, "Entry assumes the original loop guard. Earlier quality/lifecycle work and later correction work have a no-cursor-write frame; exceptions abort the host replay.")
    save("navigation_cursor", CURSOR, "Exact unconditional event accounting and rejection/continue branch. The successful correction suffix uses the separately copied final read. All numeric/status work is required before that suffix and is not assumed atomic.")
    save("navigation_advance", ADVANCE, "Exact final input advance after successful correction and check_state. The parent loop has exactly two advances, one on rejection and one on fallthrough; neither is bypassed on normal continuation.")
    MANIFEST["source_steps"]["replay_navigation_guard"]["parent_sha256"] = sha256(NAV.encode())
    START = SOURCE.index("formal_eskf::configuration::Ins::ProcessNoise<Linalg>\npredict_interval(")
    PREDICT = method(SOURCE[START:], "formal_eskf::configuration::Ins::ProcessNoise<Linalg>\npredict_interval(")
    save("prediction", PREDICT, "Whole predict_interval; same typed settings/noise fields, state/P aliases and symbolic status boundaries. Numeric producers and host diagnostic strings remain separate.")
    CONSTANTS = re.findall(r"^#define BAD_ACCEL_STD [^\n]+$", SOURCE, re.MULTILINE)
    if len(CONSTANTS) != 1:
        raise ValueError("ambiguous replay noise profile")
    save("prediction_types", CONSTANTS[0] + "\n" + method(SOURCE, "struct PredictionStatistics\n{") + ";\n",
         "Exact replay noise constant and accounting type; no duplicated profile values.")
    STATUS = method(SOURCE, "void check_status(")
    PROJECTED, COUNT = re.subn(r'throw std::runtime_error\([\s\S]*?\);', 'throw ReplayFailure{};', STATUS)
    if COUNT != 1:
        raise ValueError("unexpected replay status exception")
    PROJECTED = PROJECTED.replace("std::string_view operation", "char const * operation")
    save("status", PROJECTED, "Exact status predicate and exception transfer; diagnostic string formatting/allocation and exception RTTI are excluded.")
    MANIFEST["source_steps"]["replay_status"]["source_sha256"] = sha256(STATUS.encode())
    MANIFEST["source_steps"]["replay_status"]["exception_diagnostics_projected"] = 1


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("output_dir", type=Path)
    ARGS = PARSER.parse_args()
    try:
        generate(ARGS.output_dir)
    except (OSError, ValueError, IndexError) as ERROR:
        print(f"error: {ERROR}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
