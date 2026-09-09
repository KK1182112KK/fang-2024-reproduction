# Fang & Zhang (2024): equation-linked numerical reproduction

**Reproduction implementation and verification project: Kenshin Kotari.** The original model, inexact-predictor design and theorems are Qin Fang and Zhengqiang Zhang's. Development and documentation were AI-assisted. [CITATION.cff](CITATION.cff) cites this software separately from the source paper.

**Source [F24]:** *Inexact predictor feedback for multi-input nonlinear systems with distinct input delays*, Automatica 159 (2024), 111399, DOI [10.1016/j.automatica.2023.111399](https://doi.org/10.1016/j.automatica.2023.111399). References use the nine-page published paper: predictor **(2)–(3), p. 2**, numerical example **(35)–(40), p. 5**, Figs. **1–8, pp. 5–6**.

## Two audit layers

This repository now retains two separate evidence layers rather than rewriting the older solver history.

1. **Baseline reporting audit:** the existing Euler/nearest-grid implementations and their actual nine measured cases. This corrects earlier repository claims while leaving the baseline solvers unchanged.
2. **Extended independent audit (September 2026):** a separate original-plant implementation with timestamped held-input arrivals, an independent continuous-feedback discretization, PDF-vector figure comparisons, high-precision local characteristic roots, and small-initial-condition nonlinear tests. Read the [extended audit report](extended-audit-2026-09/REPORT.md).

The extended audit finds that the selected compensated numerical example is not fully consistent with the published compensated figures or local exponential stability at `D1=0.4, D2=0.5, D0=0.45`. In particular, the printed predictor and zero prehistory imply `U(0+)≈[-11.6011,-29.3577]`, while the vector geometry of Fig. 2 starts near `[-8.004,-20.010]`. A high-precision characteristic root is approximately `0.682101813345006 + 53.148865919078973 i`, with an original physical-plant/predictor eigenmode residual below `5e-70` in that calculation. Small-initial-condition nonlinear runs preserve the resulting growth pattern under step refinement. **This does not refute Theorem 1's abstract sufficiently-small-mismatch statement; no numerical value of its epsilon is certified.**

## Baseline equations and measured outcomes

All three existing baseline solvers advance physical equations **(35)–(36)**. The predictor is freshly computed using **(3)** and feedback **(39)–(40)**, not an autonomous stable target trajectory. This baseline audit retains the original Euler and nearest-grid history approximations and does not claim exact continuous-time reproduction.

| Scenario | Source | Recorded norm(x(10)), dt=0.001 | Interpretation |
|---|---|---:|---|
| Inexact predictor D0=0.45 | **(3), (35)–(36), (39)–(40)**; Figs. 1–2 | **0.0154599105** | Much smaller than the uncompensated response, but not the old README's approximately 1e-4. |
| No delay compensation | **(35)–(38)**; Figs. 3–4 | **23432.2793** | Large growth; completed T=10 with finite arrays, no overflow/early stop in this run. |
| Delay-free baseline | **(35)–(38)** with D1=D2=0 | **0.000294936260** | Small finite-time residual, not the old approximately 1e-5. |

[Full baseline equation-to-code map and all nine measured runs](docs/REPRODUCTION_REPORT.md) · [Measured CSV](results/reporting-audit/summary.csv)

The baseline audit also retains dt=0.002 and 0.0005 predictor runs and a five-horizon comparison at dt=0.002. At D0=0.5 that ten-second comparison ends at norm 1.31431731; it is not reported as universally small-residual convergence. A finite trajectory or fitted decay trend does not prove exponential stability or theorem applicability.

## Re-run and inspect evidence

Baseline audit:

```bash
pip install numpy scipy matplotlib pytest
python python/run_reporting_audit.py
python -m pytest python/tests -q
```

Extended independent audit:

```bash
cd extended-audit-2026-09
pip install -r requirements.txt
python audit.py
python continuous_check.py
python -m pytest -q test_audit.py
# Optional, with your own lawful copy of the paper:
python extract_figures.py --pdf YOUR_PDF
```

The baseline runner regenerates all nine full trajectory CSVs, per-case logs, endpoints, parameters and source/environment records. All cases completed locally and in the recorded [GitHub Actions audit](https://github.com/KK1182112KK/fang-2024-reproduction/actions/runs/34175324683). Full run JSON and trajectories are in that artifact, which expires after 30 days; committed summaries and the runner remain available.

**22 existing baseline Python tests passed**, with assertions unchanged. [Raw test log](results/reporting-audit/tests.log) · [Test record](results/reporting-audit/test-results.json) · [Source/environment provenance](results/reporting-audit/provenance.json).

The extended audit's archived local suite reports **17 passed** and publishes its executable source plus compact machine-readable evidence under [`extended-audit-2026-09/`](extended-audit-2026-09/). Large raw trajectories and source-derived curve CSVs remain reproducible rather than being duplicated in Git history.

Those tests do not certify the old README's stronger claims or an unconditional theorem. Default baseline terminal tests use **0.1**, and the legacy D0 sweep uses **T=30 with threshold 1.0**, not the ten-second audit comparison. Solver baseline: `c4ad9bf`; baseline audit head: `4823db9`; test-merge checkout: `03da81b918f2e9b9b0ca27390ea96fcef2ec7ee0`.

MATLAB source is retained but **was not executed by either reporting audit described here unless a specific CI record says otherwise**. Reports separate numerical refinement, figure geometry, local spectral evidence and theorem applicability. No source paper PDF or author-generated plot is redistributed, and no misconduct claim is made.

[Source specification](docs/SPEC.md) · [Actual baseline numerical methods](docs/METHODS.md) · [Reporting standard](docs/REPORTING_STANDARD.md) · [Extended independent report](extended-audit-2026-09/REPORT.md) · [Related reproduction projects](https://github.com/KK1182112KK/krstic-2016-reproduction)

Implementation code remains under the existing [MIT license](LICENSE); paper models, theorems, figures and authorship remain separately attributed.
