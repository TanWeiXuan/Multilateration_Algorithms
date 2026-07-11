# CLI and Default Simulation

## Entry Point

`src/cli/main.cpp` constructs a deterministic `TestParameters` scenario and calls `TrueRangeMultilateration::runTests`.

The default target is `(0, 0, 5)` and eight anchors occupy the corners of a 10-by-10-by-10 metre volume. Range noise has a 0.25 metre standard deviation, the seed is `42`, and each algorithm/scenario pair runs 10,000 estimates.

## Execution

The CLI first runs assertion-based checks for result aggregation and CRLB behavior, then compares every estimator under:

1. Gaussian range noise.
2. Range and anchor-position noise.
3. Range noise with simulated outliers.

Output includes mean absolute error, signed bias, maximum error, centered covariance, error second moment/MSE, and elapsed time.

## Customization

For a temporary experiment, edit the `TestParameters` assignments in `src/cli/main.cpp`. For reusable configuration or new frontend behavior, change the shared types and runners described in [Architecture](architecture.md).

Use `std::nullopt` for a nondeterministic random seed, but keep fixed seeds in regression checks. Reduce `numRuns` for faster exploratory builds.
