# Modern C++ features used in `calib_eigen_modern.cpp`

## `namespace calib`
The file wraps almost everything inside `namespace calib { ... }` to keep names like `Intrinsics`, `projectPoint`, and `homographyDLT` from colliding with other code if you later integrate this into a larger project. You can see this at the top where `namespace calib` starts and at the bottom where `} // namespace calib` ends, and then `main()` uses `using namespace calib;` to access those symbols without repeatedly writing `calib::`.

## Type aliases with `using`
At the start of the namespace, you will see `using Mat3 = Eigen::Matrix3d;`, `using VecX = Eigen::VectorXd;`, etc. These are type aliases that make the code shorter and clearer: instead of repeating long Eigen types in every function signature, the code uses `Mat3`, `Vec2`, `MatX`, `VecX`. This also makes it easier to change types later (for example, switching precision) in one place.

## Aggregate `struct` with in-class default initializers
The parameter containers `Intrinsics`, `Distortion`, and `Extrinsics` are plain `struct`s with fields like `double fx {0.0};` and `Vec3 rvec {Vec3::Zero()};`. The `{...}` values are default initializers, so the objects start in a safe state without needing constructors. This is used throughout the program—for example `Distortion initD{};` creates zero distortion, and `Extrinsics ext;` starts with zero vectors before being assigned.

## `final` on `struct`
The `struct Intrinsics final` (and similar) uses `final` to indicate it is not intended to be inherited from. This is a small clarity and design choice: these structs are meant as simple “data carriers” and not part of an inheritance hierarchy. It helps communicate intent to the reader and prevents accidental subclassing.

## Attributes: `[[nodiscard]]`
Functions like `[[nodiscard]] inline Mat3 rodriguesToR(...)` and `[[nodiscard]] inline VecX computeResiduals(...)` are marked with `[[nodiscard]]`. This tells the compiler to warn you if you call the function and then ignore the return value. In numeric code, silently ignoring a computed matrix/vector is a common source of subtle bugs, so the attribute makes mistakes more visible during compilation.

## `inline` for header-style utilities
Many small functions are defined as `inline` (for example `inline double sqr(...)`, `inline Mat3 rodriguesToR(...)`). Because everything is in one `.cpp` file this is not required for linkage, but it matches a header-style utility approach and allows the compiler to inline small hot-path functions aggressively. It also signals these are small helpers, not large stateful modules.

## `noexcept` to express “won’t throw”
Several math helpers are declared `noexcept`, such as `rodriguesToR` and `distortNormalized`. This documents that they are not expected to throw exceptions and can enable better optimization and simpler reasoning about error paths. You will notice that functions which may validate input and throw (like `homographyDLT` when sizes mismatch) do not use `noexcept`, so the code separates “pure math” from “validated API”.

## `const` correctness and references
Function parameters are often passed as `const` references (e.g., `const Vec3& X`, `const Intrinsics& K`) to avoid copies and to communicate that the function does not modify the input. You can see this pattern in `projectPoint`, `distortNormalized`, and many others. Inside functions, temporary results that should not change are declared `const`, such as `const Mat3 R = rodriguesToR(E.rvec);`, which improves readability and reduces accidental mutation.

## Templates (the `clamp` helper)
The file defines `template <typename T> [[nodiscard]] inline T clamp(...)` so the same clamping utility can work for different numeric types (though it is used mainly for `double`). It delegates to `std::clamp` and keeps the call sites consistent, like `cos_theta = clamp(cos_theta, -1.0, 1.0);` and in `clampParams(...)` for keeping focal lengths and distortion coefficients within safe ranges.

## `std::string_view` for lightweight names
The `ScopedTimer` struct stores the timer label as `std::string_view name;`. A `string_view` is a non-owning view into an existing string literal or string storage; it avoids allocating or copying strings. In this code, timers are created with string literals like `ScopedTimer t("bundle_adjust");`, and the view stays valid for the lifetime of the timer object.

## RAII (Resource Acquisition Is Initialization) with `ScopedTimer`
`ScopedTimer` is a classic RAII pattern: it captures the start time in its constructor and prints elapsed time in its destructor. In the code, blocks like `{ ScopedTimer t("homographies"); ... }` automatically time the block even if the block exits early. This removes manual start/stop calls and makes timing exception-safe and scope-based.

## `std::span` for non-owning “array views”
Several functions accept `std::span<const MatX>` or `std::span<const Extrinsics>`. A `span` is a lightweight view over contiguous memory; it carries a pointer and a length, without owning the memory. In this code it is used to pass the vector of per-view measurements into `computeResiduals` and to pass the stage masks and stage iteration counts into the LM routine, ensuring there are no accidental copies of large data structures.

## `std::array` for fixed-size stage schedules
In `main()`, the stage masks and iteration counts are stored as `const std::array<ActiveMask,3> stages { ... };` and `const std::array<int,3> iters { ... };`. `std::array` has a fixed size known at compile time, unlike `std::vector`. Here, the pipeline has exactly three stages, so `std::array` expresses that this is fixed design, not dynamic configuration.

## Lambdas for local behavior
The code uses lambdas to keep small “local logic” next to where it is used. For example, in `intrinsicsFromHomographies` there is `const auto recompute = [&](){ ... };` which recomputes intrinsics from the current `B` coefficients. In `main()`, `makeMask` is a lambda that builds an `ActiveMask` from a list of active indices. This avoids creating extra named helper functions and keeps context local.

## Move semantics (`std::move`)
In the Levenberg–Marquardt loop, when a candidate parameter update is accepted, the code does `p = std::move(pNew);`. This transfers the internal buffers of `pNew` into `p` instead of copying them, which is important because `VecX` can be large. You also see `data.imagePts[v] = std::move(obs);` in synthetic generation, moving a matrix into the vector efficiently.

## Exceptions for input validation (`throw std::runtime_error`)
Some functions validate preconditions and throw exceptions with descriptive messages, such as `homographyDLT` checking for Nx2 input and enough points. This is a deliberate choice to fail fast rather than continuing with invalid dimensions and producing NaNs. The code uses exceptions only at the boundaries where misuse is possible; the inner math routines are kept simple and often `noexcept`.

## Explicit casts and size handling (`static_cast`)
The code frequently uses `static_cast<int>(...)` or `static_cast<std::ptrdiff_t>(...)` when converting between sizes and indices, such as `const int M = static_cast<int>(imagePts.size());`. This avoids implicit narrowing warnings and makes conversions explicit, which matters when mixing Eigen sizes (often `ptrdiff_t`) with standard container sizes (`size_t`) and loop indices (`int`).

## `constexpr` layout indices and a “single source of truth”
`ParamLayout` centralizes all parameter vector offsets (`idx_fx`, `idx_k1`, `poseBase(viewIdx)` etc.) as `static constexpr` values. This is a modern C++ way to replace scattered magic numbers with named constants. When packing/unpacking the parameter vector, all indexing routes through these constants, which makes the code safer to modify and easier to review.

## `Eigen::Ref` for efficient matrix parameters
Functions such as `normalize2D(const Eigen::Ref<const MatX>& pts)` accept an `Eigen::Ref`, which is Eigen’s way of taking a reference to many compatible matrix expressions without copying them. This is especially useful when the caller might pass a block, a mapped matrix, or another expression; `Ref` can bind to it efficiently while still guaranteeing the callee sees a matrix-shaped object.

## Scoped blocks to control lifetime and intent
You’ll notice blocks like `{ ScopedTimer t("zhang_init"); ... }` and `{ ScopedTimer t("bundle_adjust"); ... }` inside `main()`. These braces create a scope so the timer is destroyed exactly at the end of that block. This is a simple but powerful C++ habit: it makes resource lifetime explicit and ties “begin/end” behavior to scope, not to manual calls.
