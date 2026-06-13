# Dependencies and Vendored Libraries

## Eigen

The repository vendors Eigen 5.0.0 under `libs/eigen-5.0.0/`.

Eigen is used for:

- `Eigen::Vector3d` and other vector/matrix types.
- Dense matrix algebra.
- `BDCSVD` least-squares solves.
- Matrix decompositions and inverses.
- Unsupported nonlinear optimization support for Levenberg-Marquardt.

## CMake interface targets

The `libs/CMakeLists.txt` file creates two intended interface targets:

- `Eigen`: exposes the main Eigen include directory.
- `EigenUnsupported`: intended to expose unsupported Eigen headers and link to `Eigen`.

## Standard library requirements

The project uses C++20 and includes features such as:

- `std::format`
- `std::optional`
- `std::function`
- Standard random-number facilities
- Standard chrono timing utilities

A compiler and standard library with working C++20 `std::format` support are required.

## Dependency update notes

When updating Eigen:

1. Replace or add the vendored Eigen directory under `libs/`.
2. Update `libs/CMakeLists.txt` include paths.
3. Rebuild and run the executable because SVD and unsupported nonlinear optimization APIs can change between versions.
4. Verify that unsupported modules are actually available through the configured include directories.
