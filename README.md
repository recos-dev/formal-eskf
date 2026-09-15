# formal-eskf

A portable C++20 error-state Kalman filter (ESKF) for attitude and inertial navigation.

Development of formal-eskf is specification-driven. It combines Lean 4 proofs, C++ verification with ESBMC, and agent-assisted review of verification evidence and traceability between specifications, proofs, and code.

## Key Concepts

- **Verifiable ESKF:** formal verification for trustworthy robotic state estimation.
- **VeriCoding-inspired development:** combine AI-assisted code and proof construction with explicit specifications and machine verification.
- **Traceability-backed quality assurance:** connect requirements, code, Lean proofs, ESBMC checks, and test evidence for agent-assisted review.
- **Portable by design:** keep a shared C++ estimator core independent of linear-algebra backends and deployment platforms.

## Build

Clone the repository:

```bash
git clone https://github.com/recos-dev/formal-eskf.git
cd formal-eskf
```

Install dependencies on Ubuntu / Debian:

```bash
sudo apt install build-essential cmake libeigen3-dev python3 python3-pip curl
python3 -m pip install -r requirements.txt
```

Build the ESKF tests and replay demos:

```bash
cmake -S . -B build/demo -DBUILD_TESTING=ON -DFORMAL_ESKF_BUILD_EXAMPLES=ON
cmake --build build/demo --parallel $(nproc)
```

## Run PX4 Replay

Replay a local ULog file and display the plots (flight logs are not bundled):

```bash
python3 examples/replay_px4.py /path/to/flight.ulg --plot
```

Or download and replay a sample from [PX4 Flight Review](https://review.px4.io/plot_app?log=e8744135-8493-4f5b-bab8-625c5b287d5a):

```bash
LOG_ID=e8744135-8493-4f5b-bab8-625c5b287d5a
curl --fail --location "https://review.px4.io/download?log=${LOG_ID}" --output "${LOG_ID}.ulg"
python3 examples/replay_px4.py "${LOG_ID}.ulg" --plot
```

## Run KITTI Replay

Replay a local KITTI Raw unsynced (`_extract`) drive and display the plots (datasets are not bundled):

```bash
python3 examples/replay_kitti.py /path/to/2011_10_03_drive_0027_extract --plot
```

Or download and replay a sample from [KITTI Raw](https://www.cvlibs.net/datasets/kitti/raw_data.php) (approximately 27 GB; only OXTS is extracted):

```bash
DRIVE_ID=2011_10_03_drive_0027
curl --fail --location \
    "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/${DRIVE_ID}/${DRIVE_ID}_extract.zip" \
    --output "${DRIVE_ID}_extract.zip"
unzip -n "${DRIVE_ID}_extract.zip" "2011_10_03/${DRIVE_ID}_extract/oxts/*"
python3 examples/replay_kitti.py "2011_10_03/${DRIVE_ID}_extract" --plot
```

OXTS GPS/INS estimates initialize and correct the filter; they are not independent ground truth.

## References

- [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508). 2017.
- [A Computationally Efficient GNSS/INS Design of Multirotor based on Error-state Kalman Filter](https://doi.org/10.23919/SICE59929.2023.10354209). 2023.
- [A benchmark for vericoding: formally verified program synthesis](https://arxiv.org/abs/2509.22908). 2025.

## License

This project is licensed under the [BSD 3-Clause License](LICENSE).
