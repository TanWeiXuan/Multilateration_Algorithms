# Test Helpers

`src/test_helpers.h` and `.cpp` provide measurement generation, anchor perturbation, result aggregation, and console output shared by the CLI and incremental simulation runner.

## Random Generation

`makeRandomEngine` uses the configured seed when present and `std::random_device` otherwise. Range helpers add Gaussian measurement noise and optional uniformly distributed positive outliers. Anchor helpers add independent Gaussian coordinate noise.

Keep deterministic seeds for tests. Validate `rangeOutlierRatio` before calling the helpers; the web UI clamps it to `[0, 1]`.

## Result Aggregation

`computeResults` returns zero-initialized results for an empty estimate set. For non-empty samples it computes:

- Mean signed error (bias).
- Mean and maximum absolute error per axis.
- Centered population covariance, `E[(e - E[e])(e - E[e])^T]`.
- Error second moment/MSE matrix, `E[e e^T]`.

The covariance and second moment are distinct when the estimator is biased. Preserve that distinction in UI labels and future exports.

## Output

`printTestParams` writes active scenario inputs. `printResults` writes bias and the selected error fields; covariance output also includes the MSE/second-moment matrix or diagonal. `computeAndPrintResults` combines aggregation and formatting.
