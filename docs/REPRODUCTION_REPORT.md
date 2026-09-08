# Fang & Zhang 2024 — equation-to-code map and measured results

**Reproduction implementation and verification project: Kenshin Kotari.**

## 1. Source, version, and scope

**[F24] Qin Fang and Zhengqiang Zhang**, *Inexact predictor feedback for multi-input nonlinear systems with distinct input delays*, Automatica **159** (2024), 111399, DOI [10.1016/j.automatica.2023.111399](https://doi.org/10.1016/j.automatica.2023.111399). Equation numbers and pages follow the inspected nine-page published version. Predictor **(2)–(3)** is on p. 2; numerical example **(35)–(40)** is in Section 6, p. 5; figures are on pp. 5–6.

This is an audit of the existing repository implementation at baseline `c4ad9bf05f2f5dff48ef4151a63df76598979180`, not author-provided code. No physical model, gain, solver or legacy test assertion was edited to obtain these values. Earlier repository statements about approximately 1e-4/1e-5, overflow and matching all eight figures are superseded by the evidence below.

## 2. Equation-to-code map

| [F24] source | Mathematical role | Implementation / actual scope |
|---|---|---|
| **(1)**; Assumptions 1–3, p. 2 | General delayed plant and nominal exponential/Lipschitz hypotheses | Theory context; no theorem-constant or admissible-mismatch certificate is supplied. |
| **(2)–(3)**, p. 2 | Common-horizon predictor and feedback | Inner predictor loop of [`run_inexact_predictor`](../python/src/inexact_predictor.py). |
| **(35)–(36)**, p. 5 | Original two-state delayed plant | Physical Euler update in `run_inexact_predictor` and [`run_uncompensated`](../python/src/uncompensated.py). |
| **(37)–(38)**, p. 5 | PD feedback using current physical state | `run_uncompensated`; [`run_delay_free`](../python/src/delay_free.py) uses D1=D2=0. |
| **(39)–(40)**, p. 5 | PD feedback evaluated at P | Control update in `run_inexact_predictor`. |
| Unnumbered predictor expansion immediately after **(40)** | Component specialization of (3) | Same inner loop; no fabricated additional source equation number. |
| **Figs. 1–2** | Predictor-compensated state/input response | Our predictor runs, no pixel-level fit. |
| **Figs. 3–4** | Delay-ignorant state/input response | Our uncompensated run, actual completion status recorded. |
| **Figs. 5–6** | Delay-free state/input response | Our delay-free run. |
| **Figs. 7–8** | Distributed actuator transport visualization | Not separately generated/quantitatively checked by this audit; not additional plant dynamics in our simulations. |

## 3. Source equations and initial conditions

The original physical equations are [F24, **(35)–(36)**]:

```math
\dot x_1(t)=x_2(t),
```
```math
\dot x_2(t)=-x_1(t)+\sin(x_1(t))+x_2(t)
+u_1(t-D_1)+\tanh(u_1(t-D_1))+\sin(u_2(t-D_2)).
```

The two controls are independent signals. For delayed scenarios, Section 6 supplies `D1=0.4`, `D2=0.5`. Gains in **(37)–(40)** are `[kp1,kd1]=[3,5]` and `[kp2,kd2]=[10,10]`. Nominal feedback is `u1=-(3*x1+5*x2)`, `u2=-(10*x1+10*x2)`; predictor feedback replaces x by P.

At every update, **(3)** is initialized at `P(t-D0)=x(t)` and integrated to t using past input history. Its component right-hand side is `[P2, -P1+sin(P1)+P2+u1+tanh(u1)+sin(u2)]`. The actual physical plant still uses its two actual delays. No future physical trajectory or independently propagated stable target is supplied to that controller.

**Source conditions in the paragraph after (40):** `x(0)=[1,1]`, `ui(tau)=0` for tau<0, and predictor horizon D0=0.45. Figs. 1–6 show the interval to T=10. The delay-free case sets D1=D2=0 as described in Section 6.

**Repository numerical choices:** Forward Euler for physical and predictor integration, default dt=0.001, additional dt=0.002/0.0005, and the D0 sweep at dt=0.002. The inspected Section 6 does not supply the 0.001 Euler implementation specification; it must not be attributed to the paper.

## 4. Implementation conventions retained

The physical grid is uniform. Inner predictor count is `N_pred=round(D0/dt)` and step `D0/N_pred`. Input history uses `round(t_query/dt)+N_delay` with array-index clamping. For the specified audit values, delays/horizons are integer multiples of dt; this does not validate arbitrary off-grid delays or histories. Predictor history uses preallocated arrays and constant zero prehistory.

Physical states advance with Forward Euler. Final predictor/control outputs are held copies rather than newly calculated terminal feedback. The uncompensated runner has an explicit `norm(x)>1e6` termination guard whose message calls it overflow; this threshold is not floating-point overflow. None of the nine cases stopped early, and no parameter was fitted to the paper figures.

These are approximations of continuous-time functional feedback, not exact delayed solutions. History lookup and integration are not silently rewritten in a reporting-only update. [METHODS.md](METHODS.md) distinguishes this method from the separate timestamp-based Krstic implementation.

## 5. Measured results: every planned case

Values are our local runs, Python 3.13.5 / NumPy 2.3.5 / SciPy 1.17.0, using unchanged baseline sources. [summary.csv](../results/reporting-audit/summary.csv) preserves every measured endpoint; [provenance.json](../results/reporting-audit/provenance.json) identifies source hashes and execution. The runner/CI artifact also retain full runs.json and trajectories. `norm` means Euclidean physical-state norm; maxima are over stored nodes, not certified continuous extrema.

### 5.1 Main scenarios, dt=0.001, T=10

| Case | Source | Final x1 | Final x2 | norm(x(10)) | Completed? |
|---|---|---:|---:|---:|---|
| Predictor, D0=0.45 | **(3), (35)–(36), (39)–(40)** | 0.000383819002 | 0.0154551453 | **0.0154599105** | Yes, finite |
| Without compensation | **(35)–(38)** | -23404.9144 | -1132.11853 | **23432.2793** | Yes, finite, no early stop |
| Delay-free baseline | **(35)–(38)**, D1=D2=0 | 0.000221066273 | -0.000195236013 | **0.000294936260** | Yes, finite |

**Result:** predictor feedback substantially reduces the ten-second response relative to the uncompensated case. The old approximately 1e-4 predictor and 1e-5 delay-free terminal norms are not reproduced. The uncompensated run grows large but does not overflow: maximum sampled state norm is **62175.5849**, below the 1e6 guard, and all arrays remain finite.

Initial input is approximately `[-11.6005611,-29.3558539]` with predictor feedback and `[-8,-20]` for the other scenarios. Maximum absolute input components in the uncompensated run are approximately **338010.734** and **724448.876**. These are recorded sample maxima, not physical actuator constraints or claims that such commands are realizable.

### 5.2 Predictor step refinement: D0=0.45, T=10

| dt | Final x1 | Final x2 | norm(x(10)) |
|---|---:|---:|---:|
| 0.002 | 0.000137739210 | 0.00854791703 | 0.00854902671 |
| 0.001 | 0.000383819002 | 0.0154551453 | 0.0154599105 |
| 0.0005 | 0.000503226478 | 0.0161409215 | 0.0161487641 |

**Result:** terminal norm increases rather than decreases as the discretization is refined here. It is not used as a convergence-order estimate. Comparing full physical trajectories to the linearly interpolated finest run gives maximum componentwise differences **0.0317465588** (dt=0.002) and **0.0108589137** (dt=0.001); see [refinement-comparison.json](../results/reporting-audit/refinement-comparison.json). A finer run is not a known exact solution or rigorous error bound.

### 5.3 Horizon comparison: dt=0.002, T=10

Other source parameters are unchanged. D0=0.45 is the same run as in the preceding table, not an extra execution.

| D0 | Final x1 | Final x2 | norm(x(10)) |
|---|---:|---:|---:|
| 0.40 | 0.00348266592 | -0.0723799827 | 0.0724637209 |
| 0.42 | 0.000414825015 | -0.000651597326 | 0.000772436967 |
| 0.45 | 0.000137739210 | 0.00854791703 | 0.00854902671 |
| 0.48 | -0.0354208617 | 0.315416969 | 0.317399594 |
| 0.50 | -0.0000831671489 | -1.31431731 | **1.31431731** |

**Result:** these finite responses have markedly different residuals. We do not describe all horizons as uniformly small-residual convergence or infer that every horizon between the two delays meets a theorem condition. The paper requires a sufficiently narrow admissible range, not merely D0 between D1 and D2. These data alone neither prove stability nor disprove the conditional theorem.

## 6. Tests, CI, and earlier claims

[GitHub Actions audit 34175324683](https://github.com/KK1182112KK/fang-2024-reproduction/actions/runs/34175324683) ran **22 existing Python tests: 22 passed**, then all nine numerical cases completed. [Raw test log](../results/reporting-audit/tests.log) and [test record](../results/reporting-audit/test-results.json) retain the outcome; full JUnit and CSVs are in the artifact.

Default convergence tests in `test_inexact_predictor.py` and `test_three_scenarios.py` use **0.1**, not 1e-4. The existing horizon sweep uses **T=30, dt=0.001 and threshold 1.0**, not this audit's ten-second comparison. `TestUncompensatedDiverges` accepts early termination or terminal norm above 1, so passing it does not establish overflow. The named predictor Euler/RK4 test actually compares two Euler resolutions; a separate delay-free test contains an RK4 comparison. Assertions were not relaxed by this audit.

CI used source head `4823db91feaacf9809a480817c7815398557c00f`, test-merge checkout `03da81b918f2e9b9b0ca27390ea96fcef2ec7ee0`, Python 3.13.15 / NumPy 2.5.3 / SciPy 1.18.1. Solver hashes match the local archive; terminal norms agree across the two environments to floating-point roundoff. This is not cross-language verification. MATLAB was not executed by this reporting audit. Later report/notebook edits are not retroactively claimed to be part of that CI checkout.

## 7. Reproduce, retain, and interpret

```bash
pip install numpy scipy matplotlib pytest
python python/run_reporting_audit.py
python -m pytest python/tests -q
```

The runner retains every planned case, actual endpoint, finite/exception status, stdout and full trajectory CSV. Hashes and environments identify what ran. The raw CI artifact has 30-day retention; committed summaries and this runner persist in Git.

No quantitative pixel fit to Figs. 1–8, exponential-stability proof, certified mismatch/RoA calculation, access to the authors' code or misconduct allegation is made. The verified finding is that existing software outcomes do not support several earlier repository claims. This report corrects the record while separating original theory attribution from reproduction work.
