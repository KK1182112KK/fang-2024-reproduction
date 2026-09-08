# Numerical methods — existing Fang reproduction

Unchanged solver baseline: `c4ad9bf05f2f5dff48ef4151a63df76598979180`. This update records actual executions, not a silent numerical repair. See [the equation-linked report](REPRODUCTION_REPORT.md).

Physical update uses (35)–(36). `run_inexact_predictor` reconstructs (3), applies (39)–(40), stores commands, then advances the plant using its two delayed inputs. `run_uncompensated` uses state laws (37)–(38) with delays retained; `run_delay_free` sets both physical delays to zero. No independent stable target generates x.

Plant and predictor use Forward Euler. `N_pred=round(D0/dt)` and spatial step D0/N_pred. Control lookup uses nearest-grid rounding with index clamping. Audit delays and steps are commensurate, but arbitrary noncommensurate histories are not certified. Negative-time inputs are zero. Terminal predictor/control outputs are held copies, not newly evaluated feedback. This is not exact continuous-time implementation or the timestamp-lookup migration of the separate Krstic project.

The uncompensated runner stops when physical-state norm exceeds 1e6. Its message uses overflow, but this is a termination threshold, not floating-point overflow. All nine measured cases reached T=10 with finite arrays; the old README's overflow description does not apply.

The audit keeps all outcomes and records CSVs, logs, source hashes and environments. Existing tests/thresholds are unchanged. A predictor test named Euler/RK4 compares fine and coarse Euler; the delay-free test separately implements RK4. Existing D0 tests use T=30 and threshold 1.0. No pixel matching, theorem-constant certificate, MATLAB execution or independent cross-language comparison was performed.

Refinement compares physical trajectories to the finest recorded trajectory using linear interpolation: maximum absolute componentwise difference at coarse sample nodes. This is not distance to a proven exact solution or a guaranteed convergence order.
