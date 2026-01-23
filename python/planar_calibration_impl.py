"""Planar camera calibration (planar target, multiple views) — beginner-friendly, heavily commented.

What this file does
-------------------
This program estimates a camera model from multiple views of a flat calibration target (a plane),
such as a checkerboard. It does not read images or detect corners. Instead, it works from two inputs:

1) Object / target points (known geometry):
   - coordinates of points on the flat target, in the target's coordinate system
   - these are 3D points, but the target is flat so the Z ("height") coordinate is always 0

2) Image measurements (observations):
   - pixel coordinates of the corresponding points in each captured view
   - the correspondence is assumed correct: point i on the target matches measurement i in the image

The output
----------
- Intrinsics (shared):
  fx, fy: focal scaling in pixel units
  cx, cy: principal point in pixels
  skew is assumed to be 0 (typical in modern cameras)

- Distortion (shared):
  k1, k2: radial distortion (symmetric effect that increases away from the principal point)
  p1, p2: tangential distortion (asymmetric effect caused by small lens/sensor misalignment)

- Extrinsics (per view):
  rvec: rotation in Rodrigues-vector form (orientation)
  tvec: translation (position)

Pipeline overview
-----------------
A) Homography per view (linear):
   Estimate a plane-to-image mapping for each view using correspondences.

B) Intrinsics initialization (linear):
   Use constraints across all views to infer the shared intrinsics.

C) Pose initialization (closed-form):
   For each view, decompose the mapping using the intrinsics to get an initial pose.

D) Reprojection refinement (iterative):
   Improve all parameters by minimizing pixel prediction error.
   Refinement is staged for stability: intrinsics+poses, then radial distortion, then tangential distortion.

Synthetic test
--------------
`main()` generates synthetic data (known ground truth + noise) and runs the pipeline end-to-end,
so you can validate behavior without any image I/O.
"""

# Postponed evaluation of type hints:
from __future__ import annotations

import logging
from dataclasses import dataclass
from collections.abc import Sequence
from typing import TypeAlias, Final

import numpy as np


# ------------------------------------------------------------
# Logging setup
# ------------------------------------------------------------
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger("planar_calibration")


# ------------------------------------------------------------
# Algorithm at a glance (pseudocode)
# ------------------------------------------------------------
# Inputs:
#   - planar target points (known geometry)
#   - measured pixel points for each view (observations)
#
# Steps:
#   1) For each view:
#        estimate a plane-to-image mapping (homography) from correspondences
#   2) From all homographies:
#        compute an initial guess for intrinsics (shared camera parameters)
#   3) For each view:
#        compute an initial pose (orientation + position) from intrinsics and homography
#   4) Refine intrinsics + distortion + all poses:
#        minimize reprojection error using staged Levenberg–Marquardt
#
# Outputs:
#   - intrinsics (fx, fy, cx, cy)
#   - distortion (k1, k2, p1, p2)
#   - pose per view (rvec, tvec)
# ------------------------------------------------------------


# ------------------------------------------------------------
# Type aliases (TypeAlias)
# ------------------------------------------------------------
Vec: TypeAlias = np.ndarray
Mat: TypeAlias = np.ndarray


# ------------------------------------------------------------
# Constants (Final)
# ------------------------------------------------------------
DEFAULT_IMG_W: Final[int] = 1280
DEFAULT_IMG_H: Final[int] = 720


# ------------------------------------------------------------
# Dataclasses (frozen=True, slots=True)
# ------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    skew: float = 0.0

    @property
    def K(self) -> Mat:
        """Computed camera matrix (built from stored numeric fields)."""
        return np.array([[self.fx, self.skew, self.cx],
                         [0.0,     self.fy,   self.cy],
                         [0.0,     0.0,       1.0]], dtype=float)


@dataclass(frozen=True, slots=True)
class Distortion:
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0


@dataclass(frozen=True, slots=True)
class Extrinsics:
    rvec: Vec
    tvec: Vec


def parse_tokens(tokens: Sequence[str] | None) -> dict[str, str]:
    """Demonstrates Sequence[...] and the | union form in type hints."""
    if tokens is None:
        return {}
    out: dict[str, str] = {}
    for tok in tokens:
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def _clamp(x: float, lo: float, hi: float) -> float:
    return float(np.clip(x, lo, hi))


def rodrigues_to_R(rvec: Vec) -> Mat:
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return np.eye(3)
    k = (rvec / theta).reshape(3)
    K = np.array([[0.0, -k[2],  k[1]],
                  [k[2],  0.0, -k[0]],
                  [-k[1], k[0], 0.0]], dtype=float)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def rvec_from_R(R: Mat) -> Vec:
    tr = float(np.trace(R))
    cos_theta = _clamp((tr - 1.0) * 0.5, -1.0, 1.0)
    theta = float(np.arccos(cos_theta))
    if theta < 1e-12:
        return np.zeros(3, dtype=float)
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]], dtype=float) / (2.0 * np.sin(theta))
    return axis * theta


def distort_normalized(xy: Vec, d: Distortion) -> Vec:
    x, y = float(xy[0]), float(xy[1])
    r2 = x * x + y * y
    r4 = r2 * r2
    radial = 1.0 + d.k1 * r2 + d.k2 * r4
    x_tan = 2.0 * d.p1 * x * y + d.p2 * (r2 + 2.0 * x * x)
    y_tan = d.p1 * (r2 + 2.0 * y * y) + 2.0 * d.p2 * x * y
    return np.array([x * radial + x_tan, y * radial + y_tan], dtype=float)


def project_point(X: Vec, K: Intrinsics, D: Distortion, E: Extrinsics) -> Vec:
    R = rodrigues_to_R(E.rvec)
    Xc = R @ X.reshape(3) + E.tvec.reshape(3)
    z = float(Xc[2])
    if abs(z) < 1e-12:
        z = 1e-12 if z >= 0.0 else -1e-12
    xy = np.array([Xc[0] / z, Xc[1] / z], dtype=float)
    xy_d = distort_normalized(xy, D)
    u = K.fx * xy_d[0] + K.cx
    v = K.fy * xy_d[1] + K.cy
    return np.array([u, v], dtype=float)


def normalize_2d(pts: np.ndarray) -> tuple[np.ndarray, Mat]:
    pts = np.asarray(pts, dtype=float)
    mean = pts.mean(axis=0)
    dif = pts - mean
    mean_dist = np.mean(np.sqrt(np.sum(dif ** 2, axis=1)))
    s = 1.0 if mean_dist < 1e-12 else (np.sqrt(2.0) / mean_dist)
    T = np.array([[s, 0.0, -s * mean[0]],
                  [0.0, s, -s * mean[1]],
                  [0.0, 0.0, 1.0]], dtype=float)
    pts_h = np.hstack([pts, np.ones((pts.shape[0], 1), dtype=float)])
    q = (T @ pts_h.T).T
    # 2:3 slicing keeps a (N,1) shape for predictable broadcasting.
    return (q[:, :2] / q[:, 2:3]), T


def homography_dlt(planarXY: np.ndarray, imageUV: np.ndarray) -> Mat:
    planarXY = np.asarray(planarXY, dtype=float)
    imageUV = np.asarray(imageUV, dtype=float)
    N = planarXY.shape[0]
    Xn, TX = normalize_2d(planarXY)
    xn, Tx = normalize_2d(imageUV)
    A = np.zeros((2 * N, 9), dtype=float)
    for i in range(N):
        X, Y = Xn[i, 0], Xn[i, 1]
        u, v = xn[i, 0], xn[i, 1]
        A[2 * i + 0, :] = [-X, -Y, -1.0, 0.0, 0.0, 0.0, u * X, u * Y, u]
        A[2 * i + 1, :] = [0.0, 0.0, 0.0, -X, -Y, -1.0, v * X, v * Y, v]
    _, _, Vt = np.linalg.svd(A)
    Hn = Vt[-1, :].reshape(3, 3)
    H = np.linalg.inv(Tx) @ Hn @ TX
    if abs(H[2, 2]) > 1e-12:
        H /= H[2, 2]
    return H


def vij(H: Mat, i: int, j: int) -> Vec:
    return np.array([
        H[0, i] * H[0, j],
        H[0, i] * H[1, j] + H[1, i] * H[0, j],
        H[1, i] * H[1, j],
        H[2, i] * H[0, j] + H[0, i] * H[2, j],
        H[2, i] * H[1, j] + H[1, i] * H[2, j],
        H[2, i] * H[2, j],
    ], dtype=float)


def intrinsics_from_homographies(Hs: Sequence[Mat]) -> Intrinsics:
    V = np.zeros((2 * len(Hs), 6), dtype=float)
    for k, H in enumerate(Hs):
        V[2 * k + 0, :] = vij(H, 0, 1)
        V[2 * k + 1, :] = vij(H, 0, 0) - vij(H, 1, 1)
    _, _, Vt = np.linalg.svd(V)
    b = Vt[-1, :]
    B11, B12, B22, B13, B23, B33 = b.tolist()
    denom = (B11 * B22 - B12 * B12)
    v0 = (B12 * B13 - B11 * B23) / denom
    lam = B33 - (B13 * B13 + v0 * (B12 * B13 - B11 * B23)) / B11
    alpha = np.sqrt(abs(lam / B11))
    beta = np.sqrt(abs(lam * B11 / denom))
    gamma = -B12 * alpha * alpha * beta / lam
    u0 = (gamma * v0 / beta) - (B13 * alpha * alpha / lam)
    return Intrinsics(float(alpha), float(beta), float(u0), float(v0), 0.0)


def extrinsics_from_KH(K: Intrinsics, H: Mat) -> Extrinsics:
    invK = np.linalg.inv(K.K)  # @property used here
    h1, h2, h3 = H[:, 0], H[:, 1], H[:, 2]
    r1p = invK @ h1
    r2p = invK @ h2
    s = 1.0 / max(1e-12, 0.5 * (np.linalg.norm(r1p) + np.linalg.norm(r2p)))
    r1 = s * r1p
    r2 = s * r2p
    r3 = np.cross(r1, r2)
    R = np.column_stack([r1, r2, r3])
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, 2] *= -1.0
        R = U @ Vt
    t = s * (invK @ h3)
    return Extrinsics(rvec=rvec_from_R(R), tvec=t.reshape(3))


def pack_params(K: Intrinsics, D: Distortion, extr_list: Sequence[Extrinsics]) -> Vec:
    M = len(extr_list)
    p = np.zeros(8 + 6 * M, dtype=float)
    p[0:4] = [K.fx, K.fy, K.cx, K.cy]
    p[4:8] = [D.k1, D.k2, D.p1, D.p2]
    for v, E in enumerate(extr_list):
        base = 8 + 6 * v
        p[base:base + 3] = E.rvec
        p[base + 3:base + 6] = E.tvec
    return p


def unpack_params(p: Vec, M: int) -> tuple[Intrinsics, Distortion, list[Extrinsics]]:
    K = Intrinsics(float(p[0]), float(p[1]), float(p[2]), float(p[3]), 0.0)
    D = Distortion(float(p[4]), float(p[5]), float(p[6]), float(p[7]))
    extr: list[Extrinsics] = []
    for v in range(M):
        base = 8 + 6 * v
        extr.append(Extrinsics(p[base:base + 3].copy(), p[base + 3:base + 6].copy()))
    return K, D, extr


def clamp_params(p: Vec, imgW: int, imgH: int) -> None:
    p[0] = _clamp(float(p[0]), 200.0, 4000.0)
    p[1] = _clamp(float(p[1]), 200.0, 4000.0)
    p[2] = _clamp(float(p[2]), 0.0, float(imgW))
    p[3] = _clamp(float(p[3]), 0.0, float(imgH))
    p[4] = _clamp(float(p[4]), -0.5, 0.5)
    p[5] = _clamp(float(p[5]), -0.5, 0.5)
    p[6] = _clamp(float(p[6]), -0.02, 0.02)
    p[7] = _clamp(float(p[7]), -0.02, 0.02)


def compute_residuals(p: Vec, object_pts: np.ndarray, image_pts_views: Sequence[np.ndarray]) -> Vec:
    M = len(image_pts_views)
    K, D, extr = unpack_params(p, M)
    N = object_pts.shape[0]
    r = np.zeros(2 * M * N, dtype=float)
    k = 0
    for v in range(M):
        R = rodrigues_to_R(extr[v].rvec)
        t = extr[v].tvec
        for i in range(N):
            X = object_pts[i]
            Xc = R @ X + t
            z = float(Xc[2])
            if abs(z) < 1e-12:
                z = 1e-12 if z >= 0.0 else -1e-12
            xy = np.array([Xc[0] / z, Xc[1] / z], dtype=float)
            xy_d = distort_normalized(xy, D)
            u = K.fx * xy_d[0] + K.cx
            vv = K.fy * xy_d[1] + K.cy
            obs = image_pts_views[v][i]
            r[k] = obs[0] - u
            k += 1
            r[k] = obs[1] - vv
            k += 1
    return r


def numeric_jacobian_active(
    p: Vec,
    active: Vec,
    object_pts: np.ndarray,
    image_pts_views: Sequence[np.ndarray],
    eps_scale: float,
) -> Mat:
    r0 = compute_residuals(p, object_pts, image_pts_views)
    m, n = r0.size, p.size
    J = np.zeros((m, n), dtype=float)
    for j in range(n):
        if active[j] == 0.0:
            continue
        step = eps_scale * max(1.0, abs(float(p[j])))
        dp = np.zeros_like(p)
        dp[j] = step
        rp = compute_residuals(p + dp, object_pts, image_pts_views)
        rm = compute_residuals(p - dp, object_pts, image_pts_views)
        J[:, j] = (rp - rm) / (2.0 * step)
    return J


def lm_phased(
    p0: Vec,
    stages: Sequence[Vec],
    iters: Sequence[int],
    object_pts: np.ndarray,
    image_pts_views: Sequence[np.ndarray],
    imgW: int,
    imgH: int,
    jac_eps_scale: float = 5e-7,
    init_lambda: float = 1e-2,
    lambda_up: float = 10.0,
    lambda_down: float = 0.3,
    max_step_norm: float = 2.5,
) -> Vec:
    p = p0.copy()
    for active, max_iter in zip(stages, iters, strict=True):
        lam = init_lambda
        for _ in range(max_iter):
            r = compute_residuals(p, object_pts, image_pts_views)
            cost = float(r @ r)
            J = numeric_jacobian_active(p, active, object_pts, image_pts_views, eps_scale=jac_eps_scale)
            JTJ = J.T @ J
            g = J.T @ r
            A = JTJ + lam * np.eye(p.size)
            try:
                dp = np.linalg.solve(A, -g)
            except np.linalg.LinAlgError:
                dp = np.linalg.lstsq(A, -g, rcond=None)[0]
            dp *= active
            if (nrm := float(np.linalg.norm(dp))) > max_step_norm:
                dp *= (max_step_norm / nrm)
            p_new = p + dp
            clamp_params(p_new, imgW, imgH)
            r_new = compute_residuals(p_new, object_pts, image_pts_views)
            cost_new = float(r_new @ r_new)
            if cost_new < cost:
                p = p_new
                lam = max(1e-12, lam * lambda_down)
            else:
                lam = min(1e12, lam * lambda_up)
    return p


def make_planar_grid(nx: int, ny: int, spacing: float) -> tuple[np.ndarray, np.ndarray]:
    XY = np.array([[i * spacing, j * spacing] for j in range(ny) for i in range(nx)], dtype=float)
    X = np.hstack([XY, np.zeros((XY.shape[0], 1), dtype=float)])
    return X, XY


def random_pose(rng: np.random.Generator) -> Extrinsics:
    rvec = rng.normal(scale=0.45, size=3).astype(float)
    tvec = np.array([rng.uniform(-0.25, 0.25),
                     rng.uniform(-0.20, 0.20),
                     rng.uniform(0.9, 1.6)], dtype=float)
    return Extrinsics(rvec=rvec, tvec=tvec)


def generate_observations(
    object_pts: np.ndarray,
    K: Intrinsics,
    D: Distortion,
    extr_list: Sequence[Extrinsics],
    noise_px: float,
    rng: np.random.Generator,
) -> list[np.ndarray]:
    views: list[np.ndarray] = []
    for E in extr_list:
        uv = np.array([project_point(X, K, D, E) for X in object_pts], dtype=float)
        uv += rng.normal(scale=noise_px, size=uv.shape)
        views.append(uv)
    return views


def rmse_reproj(
    object_pts: np.ndarray,
    image_pts_views: Sequence[np.ndarray],
    K: Intrinsics,
    D: Distortion,
    extr_list: Sequence[Extrinsics],
) -> float:
    err2 = 0.0
    cnt = 0
    for v, E in enumerate(extr_list):
        for i, X in enumerate(object_pts):
            pred = project_point(X, K, D, E)
            d = image_pts_views[v][i] - pred
            err2 += float(d @ d)
            cnt += 1
    return float(np.sqrt(err2 / max(1, cnt)))


def all_inside(image_pts_views: Sequence[np.ndarray], imgW: int, imgH: int, margin: float = 5.0) -> bool:
    for uv in image_pts_views:
        if np.any(uv[:, 0] < margin) or np.any(uv[:, 0] > imgW - margin):
            return False
        if np.any(uv[:, 1] < margin) or np.any(uv[:, 1] > imgH - margin):
            return False
    return True


def first_outside_view(image_pts_views: Sequence[np.ndarray], imgW: int, imgH: int, margin: float = 5.0) -> int | None:
    return next(
        (i for i, uv in enumerate(image_pts_views)
         if np.any(uv[:, 0] < margin) or np.any(uv[:, 0] > imgW - margin)
         or np.any(uv[:, 1] < margin) or np.any(uv[:, 1] > imgH - margin)),
        None,
    )


def main(argv: Sequence[str] | None = None) -> None:
    _ = parse_tokens(argv)
    rng = np.random.default_rng(11)
    imgW: Final[int] = DEFAULT_IMG_W
    imgH: Final[int] = DEFAULT_IMG_H
    K_gt = Intrinsics(fx=820.0, fy=800.0, cx=imgW / 2.0, cy=imgH / 2.0)
    D_gt = Distortion(k1=-0.12, k2=0.018, p1=0.0012, p2=-0.0007)
    object_pts, planarXY = make_planar_grid(nx=9, ny=6, spacing=0.04)
    M: Final[int] = 10
    extr_gt: list[Extrinsics] = []
    while len(extr_gt) < M:
        candidate = random_pose(rng)
        uv = generate_observations(object_pts, K_gt, D_gt, [candidate], noise_px=0.0, rng=rng)[0]
        if all_inside([uv], imgW, imgH, margin=10.0):
            extr_gt.append(candidate)
    image_pts = generate_observations(object_pts, K_gt, D_gt, extr_gt, noise_px=0.5, rng=rng)
    if (bad := first_outside_view(image_pts, imgW, imgH, margin=2.0)) is not None:
        logger.warning(f"View {bad} has points outside bounds; results may be unstable.")
    logger.info(f"GT reprojection RMSE (noise floor): {rmse_reproj(object_pts, image_pts, K_gt, D_gt, extr_gt):.6f} px")
    Hs = [homography_dlt(planarXY, image_pts[v]) for v in range(M)]
    K0 = intrinsics_from_homographies(Hs)
    extr0 = [extrinsics_from_KH(K0, Hs[v]) for v in range(M)]
    D0 = Distortion()
    p0 = pack_params(K0, D0, extr0)
    clamp_params(p0, imgW, imgH)
    P = p0.size

    def mask(indices: Sequence[int]) -> Vec:
        m = np.zeros(P, dtype=float)
        m[np.array(indices, dtype=int)] = 1.0
        return m

    intr = list(range(0, 4))
    rad = [4, 5]
    tan = [6, 7]
    pose = list(range(8, P))
    stage0 = mask(intr + pose)
    stage1 = mask(intr + rad + pose)
    stage2 = mask(intr + rad + tan + pose)
    p_opt = lm_phased(p0, [stage0, stage1, stage2], [6, 8, 12], object_pts, image_pts, imgW, imgH)
    K_est, D_est, extr_est = unpack_params(p_opt, M)

    def fmt(x: float) -> str:
        return f"{x: .6f}"

    logger.info("")
    logger.info("--- Intrinsics ---")
    logger.info(f"fx  GT: {fmt(K_gt.fx)}  EST: {fmt(K_est.fx)}")
    logger.info(f"fy  GT: {fmt(K_gt.fy)}  EST: {fmt(K_est.fy)}")
    logger.info(f"cx  GT: {fmt(K_gt.cx)}  EST: {fmt(K_est.cx)}")
    logger.info(f"cy  GT: {fmt(K_gt.cy)}  EST: {fmt(K_est.cy)}")

    logger.info("")
    logger.info("--- Distortion ---")
    logger.info(f"k1  GT: {fmt(D_gt.k1)}  EST: {fmt(D_est.k1)}")
    logger.info(f"k2  GT: {fmt(D_gt.k2)}  EST: {fmt(D_est.k2)}")
    logger.info(f"p1  GT: {fmt(D_gt.p1)}  EST: {fmt(D_est.p1)}")
    logger.info(f"p2  GT: {fmt(D_gt.p2)}  EST: {fmt(D_est.p2)}")

    rmse_init = rmse_reproj(object_pts, image_pts, K0, D0, extr0)
    rmse_final = rmse_reproj(object_pts, image_pts, K_est, D_est, extr_est)
    logger.info("")
    logger.info("--- Reprojection RMSE ---")
    logger.info(f"Init (homography + intrinsics init, no distortion): {fmt(rmse_init)} px")
    logger.info(f"Final (LM reprojection-error minimization):         {fmt(rmse_final)} px")


if __name__ == "__main__":
    main()
