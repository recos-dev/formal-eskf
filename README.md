# formal-eskf

A portable C++20 error-state Kalman filter for attitude and inertial navigation.

Development of formal-eskf is specification-driven. It combines Lean 4 proofs, C++ verification with ESBMC, and agent-assisted review of verification evidence and traceability between specifications, proofs, and code.

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

Build the ESKF tests and PX4 replay demo:

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

## License

This project is licensed under the [BSD 3-Clause License](LICENSE).
