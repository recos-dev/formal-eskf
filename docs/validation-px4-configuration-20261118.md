# PX4 configuration validation — 2026-11-18

Validated the uncommitted configuration changes based on formal-eskf
`da9da463aa52fb1033d30c5506cb9c528e77cec9`, against PX4 v1.16.2
`54f0455ffcd755534539a7cf33a09a20bf71d29d`, Linux/GCC 11, INS/double/PX4 matrix.
The package manifests retain the actual source/header hashes, not just the base
Git revision. Kconfiglib is pinned to
`6d9102abd55b614d3c6133529ffa40dbea7fd2cf`.

## Results

| Check | Result | Scope |
| --- | --- | --- |
| `python3 tests/test_kconfig.py` | 8 passed | Defaults match CMake and the tracked PX4 header; mode changes reach compiler flags; invalid assignments, dependencies, numbers and cross-constraints fail; configured package export/relocation retains its profile |
| `python3 integrations/px4/test_package.py --px4 ...` | 8 passed | Installation, repeated/check-only operation, ownership conflicts, source/export parity and tamper rejection |
| `make test BUILD_DIR=build/kconfig-check JOBS=2` | 44 passed | Native default-profile suite |
| Default `formal_eskf_module_test` | 127 scenarios, 0 failures | Existing production-module scenarios plus configuration wiring/behavior |
| Features-off `--configuration` | 1 scenario, 0 failures | WMM, ground effect, gravity and both bias learners disabled; changed noise, quality duration, gates and GNSS timeout |
| Tuned `--configuration` | 1 scenario, 0 failures | WMM/ground effect enabled, 1.5 m deadzone, inclination/reference checks, changed thresholds/time constants, approximate prediction/reset and full magnetic attitude gain |
| Default comparison trace | Byte-identical | Original retained fixture trace before/after, including estimator outputs; new configuration scenario is outside that comparison stream |
| PX4 executable build and installer `--check` | Passed | Final installed package restored to tracked defaults; PX4 executable rebuilt |
| C++ formatting / `git diff --check` | Passed | Formatting and whitespace |
| Offline agent-review regression suite | 100 passed | Review infrastructure fixtures; not an AI audit or formal proof |

Comparison trace SHA256:
`b842b65f87f538c15a709cc45b0869a3a6607715c772351eebc1e533ca9c10f4`.

Local evidence is retained under ignored `output/kconfig/`: selected `.config`
files, per-profile package manifests, module-test binary hashes, build/test logs,
and both comparison traces. These generated artifacts are not bundled by this
Markdown record; archive them with a release evidence set when required.

## Reproduce

```bash
make menuconfig
make px4-install PX4_DIR=/path/to/PX4-Autopilot
cmake -S /path/to/PX4-Autopilot -B /path/to/PX4-Autopilot/build/px4_sitl_formal_eskf \
    -DCONFIG=px4_sitl_formal_eskf -DFORMAL_ESKF_MODULE_TESTS=ON
cmake --build /path/to/PX4-Autopilot/build/px4_sitl_formal_eskf \
    --target formal_eskf_module_test --parallel 2
/path/to/PX4-Autopilot/build/px4_sitl_formal_eskf/formal_eskf_module_test --configuration
```

The configuration smoke fixture has stationary initialization, a barometric
ground-effect observation, and a GNSS outage. It requires initialization within
12 s and a horizontal timeout no greater than 10 s. The full default suite
retains its default-profile assumptions; arbitrary tuning needs matching input
scenarios and independent acceptance criteria.

This is native/integration evidence for the tested profiles. It does not prove
all configuration combinations, flight accuracy, target timing, or deployed
backend correctness. Shared-core numerical source was not changed. Existing
Lean/ESBMC assumptions and proof domains are not broadened by the menu.
