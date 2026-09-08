# Source specification — Fang & Zhang 2024

Equation numbers use the published nine-page Automatica 159 (2024), 111399 paper, DOI 10.1016/j.automatica.2023.111399. This specification supersedes unchecked success claims in the original draft.

| Source | Specification |
|---|---|
| (1), Assumptions 1–3, p. 2 | Delayed nonlinear system, nominal exponential stabilizability, globally Lipschitz plant and bounded feedback derivatives; not certified numerically here. |
| (2)–(3), p. 2 | One predictor over [t-D0,t], initialized from current physical state, using past input histories. |
| (35)–(36), Section 6, p. 5 | Physical two-state plant; D1=0.4, D2=0.5. |
| (37)–(40), p. 5 | PD vectors [3,5] and [10,10], acting on x or P. |
| Paragraph after (40) | x0=[1,1], zero inputs for negative times, D0=0.45; predictor component expansion is unnumbered. |
| Figs. 1–6, pp. 5–6 | Predictor, uncompensated and delay-free responses, each plotted to T=10. |
| Figs. 7–8, p. 6 | Distributed actuator visualization, not separately reproduced by this audit. |

Euler dt=0.001 is a repository choice, not a step specified by the inspected simulation section. Added steps and the D0 sweep are explicit experiments. Solvers and existing tests remain unchanged. Default terminal tests use 0.1, not the old approximately 1e-4. Source assumptions, approximations and finite observations are distinct evidence categories.

[Full report](REPRODUCTION_REPORT.md) · [All recorded cases](../results/reporting-audit/summary.csv) · [Numerical conventions](METHODS.md)
