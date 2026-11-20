#!/usr/bin/env python3
"""Extract pinned ring/context-loop steps for typed, source-compositional verification.

The original class and public methods remain byte-for-byte present. Added
nonvirtual methods introduce no data. This source correspondence is reviewed;
the generator is not a C++ refinement prover. See the verification guide.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

REPO_DIR = Path(__file__).resolve().parents[3]
# Changes to source structure, record/storage types or the complete field oracle
# require correspondence review. Do not update these solely to make a run pass.
PINS = {
    "include/formal_eskf/runtime/ring_buffer.hpp": "4e14776bcf6f1d4ba499c93a289a9ec6ceab1dc2b24173c7920f988bba9a0ac5",
    "integrations/px4/module/ObservationHistory.hpp": "324dc9679657f628c0a859b201cb5dca3f5e2f6721b35ce835b39f494f744534",
    "include/formal_eskf/linalg/matrix.hpp": "09ad7208f3d44d6686a41eb8eabc3e4963562f3f5c2d175493db1a38c402cddc",
    "formal/cpp/esbmc/support.hpp": "067c556f362de8f5ad21e8672b8711ce00311fe85f9898c8091e8ca6dbc8962e",
    "formal/cpp/esbmc/contracts.hpp": "6c5ba84e914ccb08b98acf02b204bbb07e455b166ad754f2634e6c3c65f0180c",
    "formal/cpp/esbmc/imu_owner_support.hpp": "7e33850d8ffa7c24f2cd182995804dabbd3def5cbf0429d9174a833b183f6d18",
    "formal/cpp/esbmc/observation_support.hpp": "087280558f9dface1399378630070c7a37811a0cea72d4afe719b39f14412504",
    "formal/cpp/esbmc/observation_ring.cpp": "0fab79c33184065f7641079be50d5102d8bd5de0f246c2f933fa79c3b70ea8f0",
}


def sha256(DATA):
    return hashlib.sha256(DATA).hexdigest()


def check_sources():
    for NAME, EXPECTED in PINS.items():
        if sha256((REPO_DIR / NAME).read_bytes()) != EXPECTED:
            raise ValueError(f"source correspondence requires review: {NAME}")


def extract_body(SOURCE, MARKER):
    # This deliberately handles only the exact pinned source, not general C++.
    if SOURCE.count(MARKER) != 1:
        raise ValueError(f"ambiguous source marker: {MARKER}")
    START = SOURCE.index("{", SOURCE.index(MARKER))
    DEPTH = 1
    END = START + 1
    while DEPTH:
        DEPTH += (SOURCE[END] == "{") - (SOURCE[END] == "}")
        END += 1
    return SOURCE[START + 1:END - 1]


def generate(OUTPUT_DIR):
    check_sources()
    ORIGINAL = REPO_DIR / "include/formal_eskf/runtime/ring_buffer.hpp"
    OUTPUT = OUTPUT_DIR / "formal_eskf/runtime/ring_buffer.hpp"
    if OUTPUT.resolve() == ORIGINAL.resolve():
        raise ValueError("verification output must not replace production source")
    SOURCE = ORIGINAL.read_text()
    SEARCH = extract_body(SOURCE, "for (std::size_t inspected = 0; inspected < m_count; ++inspected)")
    RETIRE = extract_body(SOURCE, "for (std::size_t retired = 0; retired < removed; ++retired)")
    POP = extract_body(SOURCE, "bool RingBuffer<Sample, Capacity>::pop_first_older_than(")
    if SEARCH.count("break;") != 1 or "break;" in RETIRE or POP.count("    m_count -= removed;") != 1:
        raise ValueError("unexpected loop or return structure")
    FINISH = POP[POP.index("    m_count -= removed;"):]
    DECLARATIONS = """
    // Verification-only entry points; no object data or original method changes.
    bool proof_search_step(std::size_t & index, std::size_t inspected,
                           std::uint64_t const & timestamp, Sample * output,
                           std::uint64_t max_age_us, bool & selected, std::size_t & removed);
    void proof_retire_step();
    bool proof_finish(std::size_t removed, bool selected);

"""
    DEFINITIONS = """
template <typename Sample, std::size_t Capacity>
bool RingBuffer<Sample, Capacity>::proof_search_step(
    std::size_t & index, std::size_t inspected, std::uint64_t const & timestamp,
    Sample * output, std::uint64_t max_age_us, bool & selected, std::size_t & removed)
{
""" + SEARCH.replace("break;", "return true;") + """
    return false;
}

template <typename Sample, std::size_t Capacity>
void RingBuffer<Sample, Capacity>::proof_retire_step()
{
""" + RETIRE + """
}

template <typename Sample, std::size_t Capacity>
bool RingBuffer<Sample, Capacity>::proof_finish(std::size_t removed, bool selected)
{
""" + FINISH + """
}

"""
    END_MARKER = "} /* end namespace formal_eskf::runtime */"
    if SOURCE.count("private:\n") != 1 or SOURCE.count(END_MARKER) != 1:
        raise ValueError("unexpected class or namespace structure")
    OVERLAY = SOURCE.replace("private:\n", DECLARATIONS + "private:\n")
    OVERLAY = OVERLAY.replace(END_MARKER, DEFINITIONS + END_MARKER)
    if OVERLAY.replace(DECLARATIONS, "").replace(DEFINITIONS, "") != SOURCE:
        raise ValueError("original source changed during extraction")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(OVERLAY)
    MANIFEST = {
        "claim": "Typed source steps; requires Lean induction and reviewed source correspondence.",
        "generator_sha256": sha256(Path(__file__).read_bytes()),
        "source_sha256": PINS,
        "original_pop_sha256": sha256(POP.encode()),
        "search_body_sha256": sha256(SEARCH.encode()),
        "retirement_body_sha256": sha256(RETIRE.encode()),
        "finish_body_sha256": sha256(FINISH.encode()),
        "overlay_sha256": sha256(OUTPUT.read_bytes()),
        "transformation": "Add data-free member methods; map search break to stop=true and fallthrough to stop=false; copy retirement body and final suffix exactly.",
    }
    (OUTPUT_DIR / "ring-steps.json").write_text(json.dumps(MANIFEST, indent=2) + "\n")


def generate_context(OUTPUT_DIR):
    SOURCE = (REPO_DIR / "integrations/px4/module/ObservationHistory.hpp").read_text()
    LOOKUP = extract_body(
        SOURCE,
        "ObservationHistory<Linalg>::lookup_context(std::uint64_t time_us, std::uint64_t maximum_age_us) const",
    )
    BODY = extract_body(LOOKUP, "for (std::size_t count = 0; count < m_context.entries(); ++count)")
    if BODY.count("continue;") != 1 or BODY.count("return {};") != 1 or BODY.count("return result;") != 1:
        raise ValueError("unexpected context control structure")
    STEP = BODY.replace("continue;", "return false;")
    STEP = STEP.replace("return {};", "output = {};\n            return true;")
    STEP = STEP.replace("return result;", "output = result;\n        return true;")
    TEXT = """// Verification-only source step, not an independent implementation.
#pragma once
#include "observation_support.hpp"
namespace formal_eskf::px4
{
inline bool context_step(observation_proof::Observations const & owner, std::size_t & index,
                         std::uint64_t time_us, std::uint64_t maximum_age_us,
                         observation_proof::Observations::Context & output)
{
    auto const & m_context = owner.m_context;
""" + STEP + "\n}\n} // namespace formal_eskf::px4\n"
    # Separate include directory: context profiles execute the original ring,
    # without accidentally importing the augmented ring-step class.
    OUTPUT = OUTPUT_DIR / "context/context_step.hpp"
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(TEXT)
    MANIFEST = {
        "claim": "Typed context source step; requires loop induction and reviewed source correspondence.",
        "generator_sha256": sha256(Path(__file__).read_bytes()),
        "source_sha256": PINS,
        "lookup_body_sha256": sha256(LOOKUP.encode()),
        "step_body_sha256": sha256(BODY.encode()),
        "extracted_sha256": sha256(TEXT.encode()),
        "transformation": "Bind the unchanged receiver member by const reference; continue returns stop=false; original returns publish output and stop=true. No arithmetic or branch changes.",
    }
    (OUTPUT_DIR / "context-steps.json").write_text(json.dumps(MANIFEST, indent=2) + "\n")


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("output_dir", type=Path, help="Verification-only include directory.")
    ARGS = PARSER.parse_args()
    try:
        generate(ARGS.output_dir)
        generate_context(ARGS.output_dir)
    except (OSError, ValueError, IndexError) as ERROR:
        print(f"error: {ERROR}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
