#!/usr/bin/env python3
"""Prepare the pinned HeightAiding source for compositional verification.

One conditional lvalue needs explicit selection on the qualified frontend.
InnovationRatio.update is a separately checked producer; its declaration keeps
the actual receiver and arguments visible to the caller harness. No production
files are modified. Reviewed correspondence is not an automated refinement proof.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

REPO_DIR = Path(__file__).resolve().parents[3]
PINS = {
    "integrations/px4/module/HeightAiding.hpp": "52e84b822359b8747c7f580990fb31e3269dc6783cc98c2435166e6c60b45c54",
    "integrations/px4/module/InnovationRatio.hpp": "6e621094546e82ee010409c31a1fa96d9ecf0d628b380ee2cee98bc0b3d36e9a",
}


def sha256(DATA):
    return hashlib.sha256(DATA).hexdigest()


def replace_once(TEXT, BEFORE, AFTER):
    if TEXT.count(BEFORE) != 1:
        raise ValueError(f"unexpected source fragment: {BEFORE}")
    return TEXT.replace(BEFORE, AFTER)


def generate(OUTPUT_DIR):
    for DIRECTORY in ("include", "integrations", "formal", "tests", "scripts", "configs"):
        if OUTPUT_DIR.resolve().is_relative_to((REPO_DIR / DIRECTORY).resolve()):
            raise ValueError("verification output must not replace source files")
    SOURCES = {}
    for NAME, EXPECTED in PINS.items():
        DATA = (REPO_DIR / NAME).read_bytes()
        if sha256(DATA) != EXPECTED:
            raise ValueError(f"source changed; review the verification boundary: {NAME}")
        SOURCES[Path(NAME).name] = DATA.decode()

    HEIGHT = SOURCES["HeightAiding.hpp"]
    HEIGHT = replace_once(HEIGHT, "source == Source::barometer ? barometer : gnss",
                          "selected_sample(source, barometer, gnss)")
    HELPER = """    static Sample const & selected_sample(Source source, Sample const & barometer, Sample const & gnss)
    {
        if (source == Source::barometer)
        {
            return barometer;
        }
        return gnss;
    }
"""
    ANCHOR = "    static constexpr std::size_t index(Source source)"
    HEIGHT = replace_once(HEIGHT, ANCHOR, HELPER + ANCHOR)

    REPORT = SOURCES["InnovationRatio.hpp"]
    SIGNATURE = "    void update(std::uint64_t sample_time_us, double innovation, double ratio)"
    if REPORT.count(SIGNATURE) != 1:
        raise ValueError("unexpected reporting producer signature")
    START = REPORT.index("{", REPORT.index(SIGNATURE))
    END, DEPTH = START + 1, 1
    while DEPTH:
        DEPTH += (REPORT[END] == "{") - (REPORT[END] == "}")
        END += 1
    DECLARATION_END = REPORT.index(SIGNATURE) + len(SIGNATURE)
    REPORT_BODY = REPORT[START:END]
    REPORT = REPORT[:DECLARATION_END] + ";" + REPORT[END:]

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUTS = {"HeightAiding.hpp": HEIGHT, "InnovationRatio.hpp": REPORT}
    for NAME, TEXT in OUTPUTS.items():
        (OUTPUT_DIR / NAME).write_text(TEXT)
    MANIFEST = {
        "claim": "Actual source caller with independently checked producer contracts and reviewed correspondence.",
        "source_sha256": PINS,
        "generator_sha256": sha256(Path(__file__).read_bytes()),
        "reporting_body_sha256": sha256(REPORT_BODY.encode()),
        "generated_sha256": {NAME: sha256(TEXT.encode()) for NAME, TEXT in OUTPUTS.items()},
        "transformation": "Select the same borrowed sample through explicit returns; replace only the reporting update body with its original declaration. Keep all fields, defaults, other bodies and evaluation order.",
    }
    (OUTPUT_DIR / "manifest.json").write_text(json.dumps(MANIFEST, indent=2) + "\n")


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
