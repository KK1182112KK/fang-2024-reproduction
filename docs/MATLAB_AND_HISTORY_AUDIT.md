# MATLAB execution, failed tests, and input-history audit

Implementation/reproducibility project: **Kenshin Kotari**. Investigation date: 2026-09-08 UTC. Original theory: Qin Fang and Zhengqiang Zhang, *Automatica* 159 (2024), 111399, DOI `10.1016/j.automatica.2023.111399`. Equation numbers refer to that published nine-page paper. Development and investigation were AI-assisted.

## Actual outcome

MATLAB was now executed, rather than inferred from Python results. **9 of 11 discovered existing MATLAB tests passed; 2 failed; none were incomplete.** All **five declared physical-plant simulation cases completed with finite arrays**. The workflow remains **failed** to preserve the test outcome. Neither original solvers nor existing assertions were changed.

- Production baseline: `14c1fc658c93f95f9ab8c43d5a0fdf19c3538361`.
- Audit harness head: `01736028b761aaee79ba278b81994908c67701d9`.
- Actual PR test-merge checkout: `8359253473bdb0b4d967edfb5c2b960e08b26532`.
- MATLAB **26.1.0.3346908 (R2026a) Update 5**.
- [Audit run 34180045689](https://github.com/KK1182112KK/fang-2024-reproduction/actions/runs/34180045689), artifact **10038599996**, `matlab-execution-audit`.
- Artifact ZIP SHA256: `c8b5e5e18753528615cac6ade40e407dcdff60b91fe9c85562271747e5244bc6`.

The artifact contains the full MATLAB diary, JUnit XML, test CSV, source hashes, checkout, metadata and five trajectory CSVs. Retention is 90 days. The compact measured record and raw test table are committed under [`results/matlab-history-audit/`](../results/matlab-history-audit/). Local Python **3.13.5** comparison runs verified all **16** source/test hashes in the MATLAB manifest before executing corresponding cases.

## 1. Failed Euler/RK4 comparison is a real test failure, not a theorem verdict

[`matlab/tests/test_cross_validation.m`](../matlab/tests/test_cross_validation.m), `test_euler_vs_rk4`, compares the production Euler implementation at dt=.001 with the test's separate RK4 implementation at dt=.005, T=10. Both attempt the predictor **(3)**, plant **(35)-(36)** and feedback **(39)-(40)**. The measured maximum Euclidean state difference on the compared grid is

```text
0.073028798253488 > 0.05
```

so the existing assertion fails. Terminal norms in the test log are approximately **0.01545991** for Euler and **0.005314412** for RK4.

The RK4 test is not a certified continuous-time reference. It recomputes feedback once per physical step, uses nearest-rounded history queries at predictor stages, and the last spatial RK4 stage can query the current preallocated zero slot before the new command is computed. Its plant stages use delayed controls at fixed t_k. Thus both discretization and feedback-update conventions must be audited before attributing the difference to one integration method. The observed 0.073 is not, by itself, proof that the paper's equations or either entire trajectory are false.

The Python test named `TestEulerVsRK4Predictor` does **not** run this same comparison. Its predictor test compares **Euler dt=.0005 against Euler dt=.002 over T=5**, using a componentwise error threshold .05. The separate Python delay-free RK4 test is also not this delayed closed-loop test. A Python pass therefore did not establish a pass for this MATLAB comparison.

## 2. Failed horizon sweep: the Python and MATLAB criteria differ

[`matlab/tests/test_three_scenarios.m`](../matlab/tests/test_three_scenarios.m), `test_D0_sweep`, requires terminal norm **<.1 at T=10**, dt=.001, for all five horizons. The production equations remain **(35)-(36)** with D1=.4, D2=.5, x0=[1,1], zero negative-time inputs, gains [3,5] and [10,10] from **(37)-(40)**.

| Predictor horizon D0 | MATLAB terminal norm at T=10 | Original MATLAB criterion |
|---|---:|---|
| .40 | 0.07536142 | pass |
| .42 | 0.001142828 | pass |
| .45 | 0.01545991 | pass |
| .48 | **0.3343695** | **fail** |
| .50 | **1.301756** | **fail** |

Values above are quoted to the precision printed in the diary. A separately archived .50 case has norm **1.3017564356442826**. The same states in Python agree to roundoff for that case.

By contrast, the existing Python horizon sweep uses **T=30 and terminal threshold <1.0**, with the same default dt=.001. It is a later endpoint and a looser threshold. This explains why the previously reported Python test pass was not a certificate of the MATLAB criterion. No test is relaxed here.

The paper's **Theorem 1 / Eq. (4), p. 3**, requires the delay-bound mismatch to be below a sufficiently small epsilon under Assumptions 1-3. It does not assert that every D0 in [.4,.5] satisfies this particular ten-second threshold. The sweep failure is a failure of **our repository's assertion**, not an established counterexample to that theorem. A nonzero finite-time norm also does not establish instability.

## 3. Full-trajectory Python/MATLAB comparison

All comparisons use the unchanged physical plant implementations; no pre-solved target state generates X or U. Error below means maximum absolute **component** difference on corresponding output nodes. It is distinct from the Euclidean metric in the failed MATLAB RK4 test. The comparison script rejects different source hashes, array shapes or time grids.

| Case | Conditions | MATLAB terminal norm | max componentwise X difference | max componentwise U difference |
|---|---|---:|---:|---:|
| Predictor | D0=.45, dt=.001, T=10 | 0.0154599105385681 | 5.32907e-15 | 4.97380e-14 |
| Uncompensated | feedback (37)-(38), actual D1=.4,D2=.5, dt=.001,T=10 | 23432.2792843544 | 5.09317e-11 | 5.23869e-10 |
| Delay-free | D1=D2=0, feedback (37)-(38), dt=.001,T=10 | 0.000294936260422552 | 4.88498e-15 | 4.97380e-14 |
| Horizon .50 | predictor D0=.5, dt=.001,T=10 | 1.30175643564428 | 7.77156e-15 | 1.24345e-13 |
| Half-grid diagnostic | **audit-only** D0=.45,dt=.02,T=.1 | 1.54788034888662 | 4.66294e-15 | **0.350363327** |

Unless overridden, x0=[1,1], negative inputs zero, D1=.4,D2=.5, gains [3,5],[10,10]. The uncompensated run was large but finite and reached T=10; it did not overflow or stop early. The standard trajectories agree across languages, including the failing-horizon case.

## 4. Half-grid roundoff conventions change predictor subdivisions

With D0=.45,dt=.02, the ratio is 22.5. Python `round` returns **22** while default MATLAB `round` returns **23**. `N_pred` therefore differs and so do the computed predictor/feedback values. The maximum componentwise input difference over T=.1 is **0.35036332676395077**. The physical states still agree at this short horizon because no nonnegative-time input has reached the plant before D1=.4; agreement of those states does not establish agreement of the controller.

These are documented language conventions, not evidence of intentional data adjustment: [Python round](https://docs.python.org/3/library/functions.html#round), [MATLAB round](https://www.mathworks.com/help/matlab/ref/double.round.html). This additional off-grid case is not a published figure condition.

## 5. Negative-time history can be replaced by a post-startup sample

The unchanged `_lookup_u` helpers in `inexact_predictor.py` and `uncompensated.py` round q/dt without first preserving the negative-time branch. The reproducible synthetic probe sets negative-time input to **1**, input at zero to **9**, dt=.01 and asks for q=-.004. Both return **9 instead of 1**. It is a concrete prehistory-boundary defect for a non-grid query.

This helper probe deliberately uses a history different from the paper's default zero history to expose the boundary. It does not establish that every default run makes this query, that future measured physical states were accessed, or that all computed curves are invalid. Exact preservation of predictor history in **(3)** and delayed inputs in **(35)-(36)** requires an explicit policy at this boundary rather than unqualified nearest rounding.

## 6. Scope and reproducibility

The audit resolves the prior MATLAB-execution unknown and identifies real test/numerical-contract discrepancies. The results concern this implementation and its tests. They do not establish an incorrect theorem, certified delay-mismatch range, exact Fig. 1-8 reproduction, or misconduct.

```bash
python python/run_history_probe.py
# Extract the cited MATLAB artifact before comparing; production source hashes must match.
python python/compare_matlab_audit.py --matlab-dir /path/to/extracted/artifact
```

The diagnostic recorder preserves false boolean fields rather than claiming a test pass. The comparison preserves the half-grid discrepancy. `addpath('matlab'); run_cross_language_audit` executes the unchanged MATLAB tests and retains evidence even when assertions fail. A future implementation fix should establish explicit causal history and sampled-control conventions, then compare refinement at matched update rates. No production solver or threshold has been changed in this investigation; this PR remains an auditable investigation, not a green validation certificate.
