# Fang & Zhang (2024): equation-linked numerical reproduction

**Reproduction implementation and verification project: Kenshin Kotari.** The original model, inexact-predictor design and theorems are Qin Fang and Zhengqiang Zhang's. Development and documentation were AI-assisted. [CITATION.cff](CITATION.cff) cites this software separately from the source paper.

**Source [F24]:** *Inexact predictor feedback for multi-input nonlinear systems with distinct input delays*, Automatica 159 (2024), 111399, DOI [10.1016/j.automatica.2023.111399](https://doi.org/10.1016/j.automatica.2023.111399). References use the nine-page published paper: predictor **(2)–(3), p. 2**, numerical example **(35)–(40), p. 5**, Figs. **1–8, pp. 5–6**.

## Equations and measured outcomes

All three existing solvers advance physical equations **(35)–(36)**. The predictor is freshly computed using **(3)** and feedback **(39)–(40)**, not an autonomous stable target trajectory. This audit retains the original Euler and nearest-grid history approximations and does not claim exact continuous-time reproduction.

| Scenario | Source | Recorded norm(x(10)), dt=0.001 | Interpretation |
|---|---|---:|---|
| Inexact predictor D0=0.45 | **(3), (35)–(36), (39)–(40)**; Figs. 1–2 | **0.0154599105** | Much smaller than the uncompensated response, but not the old README's approximately 1e-4. |
| No delay compensation | **(35)–(38)**; Figs. 3–4 | **23432.2793** | Large growth; completed T=10 with finite arrays, no overflow/early stop in this run. |
| Delay-free baseline | **(35)–(38)** with D1=D2=0; Figs. 5–6 | **0.000294936260** | Small finite-time residual, not the old approximately 1e-5. |

[Full equation-to-code map and all nine measured runs](docs/REPRODUCTION_REPORT.md) · [Measured CSV](results/reporting-audit/summary.csv)

The audit also retains dt=0.002 and 0.0005 predictor runs and a five-horizon comparison at dt=0.002. At D0=0.5 that ten-second comparison ends at norm 1.31431731; it is not reported as universally small-residual convergence. A finite trajectory or fitted decay trend does not prove exponential stability or theorem applicability.

## Re-run and inspect evidence

[Open the audit notebook in Colab](https://colab.research.google.com/github/KK1182112KK/fang-2024-reproduction/blob/main/python/notebook.ipynb)

```bash
pip install numpy scipy matplotlib pytest
python python/run_reporting_audit.py
python -m pytest python/tests -q
```

The runner regenerates all nine full trajectory CSVs, per-case logs, endpoints, parameters and source/environment records. All cases completed locally and in the recorded [GitHub Actions audit](https://github.com/KK1182112KK/fang-2024-reproduction/actions/runs/34175324683). Full run JSON and trajectories are in that artifact, which expires after 30 days; committed summaries and the runner remain available.

**22 existing Python tests passed**, with assertions unchanged. [Raw test log](results/reporting-audit/tests.log) · [Test record](results/reporting-audit/test-results.json) · [Source/environment provenance](results/reporting-audit/provenance.json).

Those tests do not certify the old README's stronger claims. Default terminal tests use **0.1**, and the legacy D0 sweep uses **T=30 with threshold 1.0**, not this audit's T=10. Solver baseline: `c4ad9bf`; audit head: `4823db9`; test-merge checkout: `03da81b918f2e9b9b0ca27390ea96fcef2ec7ee0`.

MATLAB source is retained but **was not executed by this audit**. The audit notebook runs the same declared cases rather than repeating the old unchecked narrative. The report replaces unverified all-figures-match and overflow claims, without modifying the solvers to obtain favorable numbers.

[Source specification](docs/SPEC.md) · [Actual numerical methods](docs/METHODS.md) · [Reporting standard](docs/REPORTING_STANDARD.md) · [Related reproduction projects](https://github.com/KK1182112KK/krstic-2016-reproduction)

Implementation code remains under the existing [MIT license](LICENSE); paper models, theorems, figures and authorship remain separately attributed.
