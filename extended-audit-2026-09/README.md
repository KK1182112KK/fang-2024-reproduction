# Fang & Zhang (2024): extended equation, figure, and local-stability audit

Independent audit prepared for Kenshin Kotari on 8 September 2026. Development and reporting were AI-assisted. Original model, predictor, theorem and figures remain attributed to Qin Fang and Zhengqiang Zhang.

**Read [REPORT.md](REPORT.md) before interpreting any result.** This extended audit is additive to the repository's earlier Euler/nearest-grid reporting audit; it does not overwrite that baseline. It directly advances the original delayed physical plant, adds an independent continuous-feedback discretization, checks the printed compensated figures through PDF vector geometry, computes high-precision characteristic roots of the linearized closed loop, and runs small-initial-condition nonlinear tests.

Source: *Inexact predictor feedback for multi-input nonlinear systems with distinct input delays*, Automatica 159 (2024), 111399, DOI 10.1016/j.automatica.2023.111399.

## Re-run

```bash
pip install -r requirements.txt
python audit.py
python continuous_check.py
python -m pytest -q test_audit.py
# Optional, with your own lawful copy of the published PDF:
python extract_figures.py --pdf YOUR_PDF
```

[Compact main summary](results/summary.json) | [continuous/local-stability summary](continuous_results/summary.json) | [spectrum evidence](continuous_results/spectrum.json) | [figure comparison metrics](figure_audit/comparison.json) | [publication provenance](PUBLICATION.md)

## Main measured conclusion

The selected numerical example is not fully consistent with the printed compensated figures or with local exponential stability at the published delays. A separate DOP853 initialization gives `P(0)=[1.53888738890506, 1.39688078315917]` and `U(0+)=[-11.6010660825110,-29.3576817206423]`, while the vector geometry of Fig. 2 starts near `[-8.004,-20.010]`. A high-precision characteristic root has positive real part, approximately `0.682101813345006 + 53.148865919078973 i`, and small-initial-condition nonlinear runs preserve the predicted growth under step refinement.

**Boundary:** Theorem 1 is conditional on a sufficiently small delay mismatch. This audit does not certify the theorem's epsilon and does not refute the abstract small-mismatch theorem. Findings concern the paper's selected example and its stated interpretation. No misconduct or author-code claim is made.
