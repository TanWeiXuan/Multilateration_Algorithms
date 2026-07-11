# Future Plans and Known Limitations

## High-Priority Limitations

- Public estimators still assume compatible anchor/range vector sizes and mostly finite inputs. Introduce a consistent validation and error-return policy before treating the API as production hardened.
- Validation is embedded in the CLI and uses `assert`, so Release builds may compile it out. A dedicated CTest-integrated unit-test executable should replace this arrangement.
- GitHub Pages stages web artifacts from `build-web/bin` into a separate `dist-pages` upload directory; a dedicated install/package target could make local release packaging more explicit.

## Candidate Improvements

- Separate benchmark scenarios from unit tests and register deterministic tests with CTest.
- Define a stable install/export interface for `multilat_core`.
- Add CLI argument parsing for anchors, noise, seed, run count, and selected algorithms.
- Add CSV or JSON result export and benchmark tooling that isolates solver time.
- Document mathematical derivations and validate sign/weighting conventions against published reference data.
- Add empirical confidence ellipsoids and direct CRLB comparison views.
- Add formatting/static-analysis jobs and pinned web toolchain/dependency versions.

## Maintenance Notes

- The robust nonlinear solver uses iteratively reweighted Cauchy-style weights; changes to `robustLossParam` require outlier regression coverage.
- The deterministic default seed is useful for comparisons but does not replace tests across varied geometries.
- `main` owns all code and documentation. `pages` is deployment-only; see [Branching and Releases](branching-and-releases.md).
