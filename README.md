# Planar Camera Calibration (C++20/Eigen + Python/NumPy)

This repository contains two reference implementations of a planar camera calibration pipeline (Zhang-style initialization + nonlinear refinement), written without computer-vision libraries:

- C++20 + Eigen implementation (fast, self-contained, includes synthetic harness)
- Python + NumPy implementation (readable, useful for cross-checking numerics)

The calibration model is a pinhole camera with Brown–Conrady distortion:
- Intrinsics: fx, fy, cx, cy (skew assumed 0)
- Distortion: radial (k1, k2) and tangential (p1, p2)
- Extrinsics: per-view pose (R, t) (Rodrigues rotation vector + translation)

## Repository layout

- `planar_calibration_method.tex`  
  Method write-up shared by both implementations.

- `cpp/`
  - `planar_calibration_impl.cpp` — C++20 + Eigen implementation (includes a synthetic test harness in `main()`).
  - `planar_calibration_language_notes.md` — Notes on modern C++ usage in the implementation.
  - `planar_calibration_sweep10_report.md` — 10-trial synthetic sweep report for the C++ implementation.

- `python/`
  - `planar_calibration_impl.py` — NumPy-only implementation (includes a synthetic test harness).
  - `planar_calibration_language_notes.md` — Notes on modern Python usage in the implementation.
  - `planar_calibration_sweep10_report.md` — Synthetic sweep report for the Python implementation.

## What this repo is (and is not)

### Included
- Planar calibration pipeline:
  1) homography estimation (normalized DLT)
  2) intrinsic initialization from multiple homographies (Zhang constraints)
  3) pose initialization per view (homography decomposition)
  4) nonlinear refinement (phased Levenberg–Marquardt on reprojection error)

- Synthetic evaluation:
  - generates a planar grid, random valid poses, adds pixel noise
  - reports reprojection RMSE and parameter errors against known reference parameters

### Not included
- Corner/feature detection (checkerboard/Charuco detection, subpixel refinement, etc.)
- Real-image I/O, image processing, or any OpenCV-style utilities
- Non-planar calibration targets or multi-camera calibration
- Robust loss functions (Huber/Cauchy), outlier rejection, bundle-adjustment tooling beyond LM

## Dependencies

### C++ (Eigen)
You need Eigen headers available on your system.

- macOS (Homebrew): `brew install eigen`
- Ubuntu/Debian: `sudo apt-get install libeigen3-dev`

### Python
- Python 3.10+ recommended
- NumPy required: `pip install numpy`

## Quick start (C++)

From repo root:

```bash
clang++ -O2 -std=c++20 cpp/planar_calibration_impl.cpp -I /usr/include/eigen3 -o calib_cpp
./calib_cpp
```

Notes:
- The include path may differ by OS/package manager.
- `cpp/planar_calibration_impl.cpp` contains a synthetic harness in `main()` that prints recovered parameters and RMSE.

## Quick start (Python)

From repo root:

```bash
python3 python/planar_calibration_impl.py
```

The Python script runs a synthetic harness and prints recovered parameters and reprojection RMSE.

## Inputs for real data (expected format)

Both implementations are structured around the same mathematical inputs:

- Object points on the plane (target coordinate system), e.g. [(X, Y, 0), ...]
- Image points per view, e.g. [(u, v), ...] for each captured image
- Point-to-point correspondence must be known (same indexing)

This repository deliberately does not include detection; to calibrate from real images you must supply:
- the planar target point coordinates (known geometry), and
- the detected 2D locations (from any external detection pipeline you trust)

## Output parameters

- Intrinsics: fx, fy, cx, cy
- Distortion: k1, k2, p1, p2
- Per-view extrinsics: Rodrigues rvec + translation tvec
- Fit metric: reprojection RMSE (pixels)

## Known limitations / failure modes

- Planar-only: degeneracies occur with insufficient pose diversity (small tilts, narrow depth range).
- Non-convex refinement: LM can converge to a local minimum depending on initialization and numeric Jacobian stability.
- Numeric Jacobians: finite differences can introduce sensitivity to step size and parameter scaling.
- Skew fixed to zero: if your sensor has meaningful skew, the model must be extended.
