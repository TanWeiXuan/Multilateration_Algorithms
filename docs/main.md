# Executable and Simulation Entry Point

## File

`src/main.cpp`

## Purpose

`main.cpp` is the command-line entry point for the repository. It creates a fixed simulation configuration and passes it to `TrueRangeMultilateration::runTests`.

## Default scenario

The default configuration estimates a target at:

```cpp
Eigen::Vector3d(0.0, 0.0, 5.0)
```

using eight anchors placed at the corners of a rectangular 3D volume:

- Four anchors at `z = 10.0`.
- Four anchors at `z = 0.0`.
- Each layer uses x/y coordinates of `-5.0` and `5.0`.

The default noise and Monte Carlo settings are:

- Range noise standard deviation: `0.25` meters.
- Random seed: `42`, making runs reproducible.
- Number of runs: `10,000` per algorithm per scenario.
- Initial range outlier ratio: `0.0`.
- Range outlier magnitude: `100.0`.

## Output

The executable prints a title, runs all test scenarios, and writes formatted error and timing summaries to standard output.

## How to customize

To experiment with different scenarios, edit the `TestParameters` assignments in `main.cpp`:

- Change `truePosition` to move the target.
- Add, remove, or reposition `anchorPositions`.
- Tune `rangeNoiseStdDev`, `anchorPosNoiseStdDev`, `rangeOutlierRatio`, and `rangeOutlierMagnitude`.
- Set `randomSeed` to `std::nullopt` for nondeterministic random-device seeding.
- Reduce `numRuns` for faster local iteration.
