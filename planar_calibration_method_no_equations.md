# Planar calibration algorithm description (no equations)

Planar camera calibration in this repository is a procedure that estimates a camera’s internal parameters and lens distortion by observing a flat target from multiple viewpoints. The target can be a checkerboard, dot grid, or any printed pattern on a rigid flat surface, as long as you know the target’s point coordinates in the target’s own coordinate system and you can provide the corresponding measured pixel coordinates in each image. The output is a set of camera parameters that can predict where points on the target should appear in the image, plus a pose estimate for each captured image showing where the camera was relative to the target.

The calibration is “planar” because the target’s points all lie on a single plane. That one geometric assumption changes the problem in an important way: when a plane is viewed through a perspective camera, the entire mapping from plane coordinates to image pixels can be represented by a single projective transformation for each image. This per-image transformation is the bridge between raw correspondences and the camera’s internal parameters. The algorithm exploits that bridge to produce a strong initial estimate, then improves it by directly minimizing pixel-level prediction errors.

## Camera model being estimated

The repository uses a pinhole camera model with lens distortion. “Pinhole model” here means the camera is treated as an ideal perspective projection: rays pass through a single viewpoint, then intersect an image plane. The internal parameters being estimated are:

- Horizontal focal scaling: how strongly the camera maps horizontal angular changes into pixel changes. This is often close to the vertical value but can differ.
- Vertical focal scaling: the same concept along the vertical direction.
- Principal point horizontal coordinate: the pixel coordinate that corresponds to the optical axis intersection with the sensor, along the horizontal direction.
- Principal point vertical coordinate: the same concept along the vertical direction.
- Skew: a coupling term between horizontal and vertical pixel axes. This repository assumes it is negligible and treats it as fixed at zero.

In addition, the algorithm estimates lens distortion using a commonly used Brown–Conrady style model with four coefficients:

- Two radial distortion coefficients: these model symmetrical bending of rays that depends mainly on distance from the image center. Barrel and pincushion behaviors are the typical effects captured by these terms.
- Two tangential distortion coefficients: these model asymmetrical effects caused by decentering or slight lens misalignment relative to the sensor.

Finally, for each image, the algorithm estimates the camera pose relative to the target:

- Orientation: how the camera is rotated relative to the target plane.
- Position: how the camera is translated relative to the target plane.

These per-image pose parameters are needed because each image is taken from a different viewpoint, and calibration needs to separate “what belongs to the camera” from “what belongs to the viewpoint.”

## Inputs the method expects

For each captured image, the method expects a set of paired data:

1. Target coordinates: the known coordinates of each target point on the plane. These are usually given in target units such as millimeters or meters. The target coordinate system is chosen by you; it just needs to be consistent across all images.
2. Image coordinates: the measured pixel coordinates of the same points in the image. These are typically produced by a corner or blob detector. This repository does not implement detection; it assumes you provide these pixel measurements.
3. Correspondence ordering: the method assumes that target point number one matches image measurement number one, target point number two matches image measurement number two, and so on. If the ordering is wrong, the calibration will fail.

## Stage 1: Estimate one plane-to-image mapping per image

The first major stage estimates a single projective mapping per image that best maps the target plane points to the observed pixel points, ignoring distortion during this step. This mapping is called a homography. Conceptually, it is a flexible transformation that can represent perspective effects on a plane, including tilt and foreshortening.

To estimate this mapping, the algorithm builds a linear system from the point correspondences in that image. Each correspondence contributes constraints that say: “when the target point is transformed by the mapping, it should land on the observed pixel location.” Solving this linear system gives the mapping parameters.

A crucial numerical detail is normalization. Raw pixel coordinates may be on the order of hundreds or thousands, while target coordinates might be fractions or tens depending on your units and grid spacing. Linear solvers are sensitive to scale differences like that. The method therefore normalizes the point sets before solving:

- It shifts the points so their average is near the origin.
- It scales them so typical distances from the origin have a consistent magnitude.

This is done separately for target coordinates and pixel coordinates. After solving in the normalized coordinate systems, the algorithm converts the solution back into the original coordinate systems. This normalization makes the per-image mapping estimation much more stable, especially when measurements are noisy.

After this step, the algorithm has one homography per image. Each homography summarizes how the plane appeared in that specific image.

## Stage 2: Initialize the camera’s internal parameters from multiple mappings

A single homography mixes camera internals and viewpoint. You cannot uniquely recover camera intrinsics from one image alone because a different camera combined with a different pose could produce the same plane-to-image mapping. The method becomes solvable when you have multiple images from different poses, all taken with the same camera.

The algorithm uses a property of real camera motion: the orientation matrix describing a camera’s rotation has orthonormal column vectors. In plain terms, the camera’s local coordinate axes are perpendicular and have consistent scale. When you write the homography in terms of camera internals and pose, that orthonormality implies constraints that depend only on the intrinsic parameters, not on the translation. Each image contributes two independent constraints of this kind. With enough images, you get an overdetermined system where the intrinsic parameters are the shared values that best satisfy all constraints at once.

Operationally, the method stacks these constraints from all images and solves a linear system that yields an intermediate representation of the intrinsics. From that intermediate representation, it reconstructs the focal scalings and principal point. Because skew is assumed negligible, the reconstruction is simplified and more stable under typical conditions.

At the end of this stage, the algorithm has an initial intrinsic calibration. Lens distortion is still set to zero at this point because distortion makes the mapping nonlinear, and the goal here is a reliable starting point.

## Stage 3: Initialize the pose for each image

With an initial intrinsic estimate, the method can compute an initial pose for each image. The homography for an image can be interpreted as the intrinsic matrix multiplied by a matrix that contains the first two orientation axes and the translation, up to a scale factor. If you remove the intrinsic effects from the homography, what remains is proportional to those pose components.

The procedure is:

- Use the inverse of the estimated intrinsics to “normalize” the homography. This moves from pixel units into a normalized camera coordinate system.
- Extract three vectors from that normalized transform. Two of them correspond to the first two orientation axes up to scale, and the third corresponds to translation up to the same scale.
- Choose the scale so that the first orientation axis has unit length, then apply the same scale to the second axis and translation.
- Compute the third orientation axis as a cross product of the first two, which enforces a right-handed coordinate system.
- Because measurement noise and linear estimation can produce a matrix that is close to, but not exactly, a valid rotation, the method projects it onto the nearest valid rotation. This is typically done by a decomposition that finds the closest orthonormal matrix.

This gives a consistent initial orientation and translation for each image, compatible with the initial intrinsics.

## Stage 4: Refine everything by minimizing pixel reprojection error

The final and most important stage is nonlinear refinement. The algorithm now has a complete initial guess: intrinsics, per-image pose, and distortion initialized to zero. It refines these parameters by minimizing reprojection error, which is the difference between:

- the pixel location predicted by the camera model for a target point, and
- the pixel location actually observed for that point.

The predicted pixel location is obtained by:

1. Transforming the target point from target coordinates into camera coordinates using the pose for that image.
2. Converting that camera-space point into normalized image coordinates by perspective projection, which depends on the point’s depth.
3. Applying lens distortion to those normalized coordinates.
4. Converting the distorted normalized coordinates into pixel coordinates using the intrinsic parameters.

The objective function sums squared errors over all points in all images. This directly matches the calibration goal: produce parameters that predict measured pixels as closely as possible.

The optimizer is a Levenberg–Marquardt method, which behaves like a blend between Gauss–Newton and gradient descent. It repeatedly:

- computes the residual errors for the current parameters,
- estimates how those residuals change if parameters change slightly,
- proposes an update step that should reduce the total squared error,
- and uses a damping mechanism to decide whether to take that step and how aggressive the next step should be.

A practical implementation detail is how the method estimates sensitivity of residuals to parameter changes. In this repository, the derivatives are obtained numerically using finite differences. That means the algorithm perturbs one parameter slightly, recomputes all residuals, and uses the difference to approximate how residuals change with that parameter. This approach is straightforward and keeps the code self-contained, but it creates two responsibilities:

- Choosing a good perturbation size. Too small can amplify numerical noise; too large can bias the derivative approximation.
- Scaling different parameters sensibly. Focal lengths and principal point are measured in pixels, while distortion coefficients are dimensionless and often small; steps must be scaled accordingly.

To manage stability, the implementation uses relative step sizing, update caps, and parameter bounds to prevent unphysical values during optimization.

## Phased activation of parameters

To reduce the chance of unstable behavior, the refinement is run in phases:

- First phase: optimize intrinsics and per-image poses while distortion is held at zero. This lets the solver lock in the dominant geometry.
- Second phase: enable the two radial distortion coefficients and continue optimizing intrinsics and poses. Radial distortion usually accounts for the largest lens deviations, so adding it next yields significant improvement.
- Third phase: enable tangential distortion coefficients and continue optimization. Tangential effects can be more subtle and can correlate with principal point and pose, so enabling them last is typically more stable.

This phased approach reduces strong coupling early in the optimization and improves convergence reliability.

## Synthetic testing and ground truth comparison

Both implementations include a synthetic testing harness to validate correctness. The harness:

- Creates a planar grid of target points with known spacing.
- Chooses reference intrinsics and reference distortion coefficients.
- Samples multiple camera poses and rejects poses where points would fall outside the image bounds (with a margin), so that all points are visible.
- Projects target points into pixel coordinates using the reference camera model.
- Adds random pixel noise to simulate measurement error.
- Runs the full calibration pipeline from scratch.
- Compares estimated parameters to the reference parameters and reports reprojection errors.

Two reprojection error quantities are especially informative:

- Noise-limited baseline error: the reprojection error you get when you use the reference parameters against the noisy observations. This is the best you can reasonably expect, because the observations have noise.
- Final refined error: the reprojection error after optimization. When this value is close to the baseline, the calibration is effectively extracting all available information from the noisy measurements.

The sweep reports in the repository repeat this process across multiple trials where the reference intrinsics change each time. This shows typical performance and exposes occasional failure cases.

## Limitations and expected failure modes

Even with careful initialization, this problem is non-convex. Occasional convergence to a local minimum can occur. Common causes include:

- Insufficient viewpoint diversity: if images are too similar, the algorithm struggles to separate focal scaling, principal point, and distortion.
- Poor point measurements or incorrect correspondences: mismatched points can dominate the objective and mislead optimization.
- Numeric Jacobian sensitivity: finite difference derivatives can become noisy if step sizes are not well tuned.
- Strong coupling between distortion and intrinsics: distortion can partially mimic changes in focal scaling or principal point if the data do not constrain them well.

Standard mitigations include increasing iteration budgets, improving derivative accuracy, adding restarts from slightly perturbed initial conditions, enforcing stronger pose diversity during data collection, and using robust losses when outliers exist.

## Summary

The method works by converting a planar calibration problem into a set of per-image projective mappings, using those mappings to derive a strong initial estimate of the camera’s internal parameters, recovering per-image poses, and then refining all parameters to directly minimize pixel prediction error under a realistic camera and distortion model. The repository’s C++ and Python implementations follow the same method, and the included synthetic harnesses provide a controlled way to test accuracy against known reference parameters.
