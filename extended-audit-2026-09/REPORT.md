# Fang & Zhang (2024): independent equation, figure, and local-stability audit

**Prepared for Kenshin Kotari — 8 September 2026.**

The original model, predictor and theorem belong to Qin Fang and Zhengqiang Zhang. This project contributes an independent computational audit, not authorship of their theory. Code, analysis and reporting were AI-assisted in this session. All numerical results below were actually computed in a local Python container. Neither MATLAB, Google Colab nor GitHub Actions execution is claimed for the archived local run; this repository may additionally run the committed audit through CI.

## Executive finding

**The printed numerical example is not fully consistent with the published compensated figures or the claimed exponential convergence at the selected delays.** This is not based solely on a small nonzero endpoint.

1. The predictor in Eq. (3), the zero input prehistory and the stated state `[1,1]` uniquely determine the initial predictor and commands. A separate DOP853 calculation gives `P(0)=[1.53888738890506, 1.39688078315917]` and `U(0+)=[-11.6010660825110,-29.3576817206423]`. Vector geometry extracted from Figure 2 instead starts near `[-8.004,-20.010]`, close to the *unpredicted* feedback `[-8,-20]`. This is a source-formula/figure discrepancy, not an inferred account of the authors' program.
2. The linearization at the origin, using the printed gains and delays, has positive-real-part characteristic roots. One high-precision root is `0.682101813345006 + 53.148865919078973 i`. The root also satisfies the original physical-plant/predictor integral relation; it is not merely an extraneous root of a differentiated auxiliary model.
3. Independently recomputed nonlinear trajectories with initial state `[1e-6,1e-6]` and zero prehistory grow to a state-norm peak near `0.0505`. Halving the continuous-feedback discretization step preserves this behavior.

**Boundary of the conclusion:** Theorem 1 is conditional on a sufficiently small admissible delay mismatch. It does not assert that every choice between 0.4 and 0.5 is valid. No value of its epsilon is certified here. The findings concern the paper's selected example and its stated interpretation; they do not refute the abstract small-mismatch theorem.

## 1. Source and equation map

**[F24]** Q. Fang and Z. Zhang, *Inexact predictor feedback for multi-input nonlinear systems with distinct input delays*, Automatica 159 (2024), 111399. DOI: **10.1016/j.automatica.2023.111399**. The source is the supplied nine-page published PDF, not a draft with assumed renumbering.

| Source location | What was implemented or checked |
|---|---|
| Eq. (1), Assumptions 1–3, p. 2 | Nonlinear model class, nominal exponential stability, global Lipschitz and feedback-gradient hypotheses; not a complete theorem-proof audit |
| Eqs. (2)–(3), p. 2 | Common-horizon predictor, initialized afresh at the current physical state at every update |
| Theorem 1, Eq. (4), p. 3 | Conditional small-mismatch guarantee; not automatically certified by choosing the midpoint |
| Eqs. (35)–(36), p. 5 | Original delayed physical plant, directly time-stepped |
| Eqs. (37)–(40), p. 5 | Current-state feedback, predictor feedback and specified gains |
| Figs. 1–6, pp. 5–6 | Quantitative comparison against extracted PDF vector paths |
| Eqs. (6)–(8), Figs. 7–8 | Transport fields reconstructed from issued inputs, not independently solved by a second PDE discretization |
| Lemma 1 and Appendix (A.1)–(A.6) | Independent endpoint differentiation identity check on smooth manufactured signals |

The physical plant is

```math
\dot x_1=x_2,
```
```math
\dot x_2=-x_1+\sin x_1+x_2+u_1(t-0.4)+\tanh u_1(t-0.4)+\sin u_2(t-0.5).
```

The paper's predictor has horizon `D0=0.45` and solves, at each physical time `t`,

```math
\frac{dp}{ds}=f(p(s),U(s)),\qquad p(t-D_0)=x(t),\qquad P(t)=p(t),
```
```math
U_1(t)=-3P_1(t)-5P_2(t),\qquad U_2(t)=-10P_1(t)-10P_2(t).
```

Both actual delayed inputs remain in the physical plant. No stable target is integrated to manufacture physical state data.

## 2. Two independent numerical formulations

### 2.1 Sampled-data, held-command audit (`audit.py`)

Only the physical state is time-stepped. At each sampling instant, the predictor is recomputed by RK4 over the previously issued zero-order-held input history. A partially covered history bin is integrated over its actual length. The new command is stored and held over the next sample interval. Physical RK4 intervals are split at each delayed command arrival; the actual delay is not rounded to the nearest grid point. Negative-time inputs are exactly zero, and the startup jump is not smeared backward. Terminal commands are recomputed, not copied from the preceding node.

This is a sampled-data approximation, **not** an exact continuous-time feedback law. RK4 inside the simulator does not make the complete held-feedback approximation fourth order in the sample period. An Euler comparison is explicit; it reproduces the prior repository endpoint `0.0154599105386` at step 0.001 without changing the source equations.

### 2.2 Continuous-feedback cross-check (`continuous_check.py`)

A separate method-of-steps discretization uses piecewise-linear *past* command history. Each physical node first receives a state computed from already available delayed commands. The predictor integral is then reconstructed and the current command's contribution is solved by fixed-point iteration, with a scaled stopping tolerance of `2e-13` and a hard 40-iteration limit. The physical step and the predictor use RK4 with linearly varying forcing. All tested delays are integer multiples of this method's step; unsupported noncommensurate settings raise an error rather than being rounded.

This interpolation is an offline numerical approximation of continuous functional feedback, not an online controller that knows the next command. Negative-time history remains zero on bins ending at zero. No future physical measurement or independent target is supplied to the predictor.

The largest observed final fixed-point update was below `6e-12`. That is an iterative-solve diagnostic, not a bound on total DDE error.

## 3. Main numerical results

For the continuous-feedback cross-check, original paper conditions, `T=10`:

| Step (s) | Physical norm at 10 s | Maximum componentwise trajectory difference from step 0.0005 |
|---:|---:|---:|
| 0.005 | 0.0143775275 | 0.00150080793 |
| 0.002 | 0.0146192334 | 0.000229314080 |
| 0.001 | 0.0146510545 | 0.0000458696908 |
| 0.0005 | 0.0146589049 | reference discretization, not an exact solution |

The trajectory differences reduce systematically. A residual alone would not demonstrate instability; the local spectrum and small-state runs below are the stronger checks.

The sampled-data audit also executes uncompensated and delay-free cases, a horizon sweep, an equal-delay calibration, a 30-second run and noncommensurate delays. All 18 declared main cases completed. All seven continuous-feedback/linearized cases completed. Two additional Euler baseline runs support the figure comparisons. Every case is retained, including unfavorable outcomes; no gain or delay was fitted to the graphics.

## 4. Why the selected example is not locally exponentially stable

Linearizing the *printed* physical model and feedback at the origin gives

```math
A=\begin{pmatrix}0&1\\0&1\end{pmatrix},\quad C_1=B_1K_1=\begin{pmatrix}0&0\\-6&-10\end{pmatrix},\quad C_2=B_2K_2=\begin{pmatrix}0&0\\-10&-10\end{pmatrix}.
```

In particular, the coefficient 2 in `B1` is from the derivative of `u1+tanh(u1)` at zero; the derivative of `sin(u2)` is 1. The characteristic matrix derived from the physical plant and predictor is

```math
M(s)=sI-A-C_1-C_2-e^{AD_0}\left[C_1(e^{-sD_1}-e^{-sD_0})+C_2(e^{-sD_2}-e^{-sD_0})\right].
```

High-precision roots of `det M(s)=0`, with 70 decimal-digit arithmetic, include

```math
s_1\approx 0.682101813345006+53.148865919078973i,
```
```math
s_2\approx 0.606723539419281+39.448447922497325i.
```

For an independent consistency check, define

```math
Q(s)=\int_0^{D_0}e^{(A-sI)r}(C_1+C_2)\,dr.
```

For a null vector `p` of `M(s)`, reconstruct `x=e^{-AD0}(I-Q(s))p` and substitute it into

```math
(sI-A)x=(C_1e^{-sD_1}+C_2e^{-sD_2})p.
```

The largest residual for the first reported mode is about `4.5e-70` in the high-precision calculation. This checks the original integral predictor relation as well as the physical plant. The search is not a complete computation of the spectrum and is not an interval-arithmetic certificate. The positive roots nevertheless provide direct numerical evidence incompatible with local exponential stability at these parameters.

### Small initial-state experiment

Both nonlinear runs use `x(0)=(1e-6,1e-6)` and zero input prehistory; this is an explicitly declared audit condition, not the paper's plotted initial condition.

| Window (s) | Peak physical norm, step 0.001 | Peak physical norm, step 0.0005 |
|---|---:|---:|
| 5–10 | 0.0000548075 | 0.0000549587 |
| 10–15 | 0.00155406 | 0.00156055 |
| 15–20 | 0.0342233 | 0.0343386 |
| 20–25 | 0.0505487 | 0.0505365 |

The initial norm is about `1.4142e-6`. The step-0.001 run extends to 35 s; the finer run extends to 25 s. The linearized run through 15 s gives the same initial growth scale before nonlinear effects become appreciable. Finite nonlinear trajectories do not establish a limit cycle or their ultimate behavior.

## 5. Direct comparison with the paper's graphics

`extract_figures.py` reads the actual PDF line segments via PyMuPDF. The rendered pages were inspected to calibrate tick values. No OCR, screenshot tracing or parameter fitting was used. The extraction records the PDF SHA-256, page, path indices, axes and coordinate transform. The source PDF is not redistributed. Extracted curves are **publication graphics, not author raw numerical data**.

On a uniform 0.01-second comparison grid:

| Figure | Comparison | Maximum absolute discrepancy, components 1 / 2 |
|---|---|---:|
| 1: compensated state | Continuous-feedback step 0.0005 | 0.170225 / 0.438709 |
| 2: compensated input | Continuous-feedback step 0.0005 | 3.99552 / 9.45977 |
| 3: uncompensated state | Euler step 0.001 | 8.34 / 18.18 on axes of order `1e4` |
| 4: uncompensated input | Euler step 0.001 | 94.0 / 175.9 on axes of order `1e5` |
| 5: delay-free state | Euler step 0.001 | 0.0000776 / 0.000323 |
| 6: delay-free input | Euler step 0.001 | 0.00222 / 0.00368 |

The uncompensated and delay-free baselines closely track the published graphics at their plotting scales. Large-axis vector quantization makes their tiny initial values unreadable numerically; their extracted startup values are not treated as precise data. Conversely, the compensated discrepancies are much larger than the refinement changes and plotting-coordinate resolution. Figure 2's finite, visible initial coordinates do not agree with the uniquely initialized Eq. (3) predictor.

This does **not** identify the authors' implementation. For example, an initialization convention could be involved, but it is not silently substituted for the printed integral equation.

## 6. Checks, provenance and execution

The executed suite reports **17 passed** in `tests.log`, with JUnit details in the full archive. Checks cover unforced pre-arrival trajectories against DOP853; independently initialized predictors; equal-delay agreement with future *physical* state; terminal command recomputation; horizon-prefix independence; off-grid physical replay; continuous-feedback refinement; the differentiated predictor identity; the spectral mode's original-system residual; and parameter rejection.

Passing these tests is not a blanket theorem certificate. The off-grid replay is a second plant integrator using an already issued command sequence, not an independently closed-loop controller. The equal-delay sampled run has prediction/physical-state discrepancy around `1.7e-14` for that finite experiment; this is not a machine-precision claim for the unequal-delay system.

Source hashes, environments, endpoint summaries, extraction settings and logs accompany the full project archive. Earlier Euler-only repository reports are preserved rather than overwritten. This independent audit adds source-figure and local-spectrum checks that the earlier report did not perform.

## 7. Reproduce on a CPU

The full supplied audit archive includes a Colab notebook. In this compact GitHub publication, the scripts can be run directly:

```bash
pip install -r requirements.txt
python audit.py
python continuous_check.py
python -m pytest -q test_audit.py
# Optional: use your lawfully obtained copy of the supplied PDF.
python extract_figures.py --pdf "Inexact 1(3).pdf"
```

`continuous_check.py` resumes completed per-case CSV/JSON pairs after confirming their parameters. For a fresh independent rerun, use a clean working copy or remove the generated output directories. Figures 7–8 in the full audit archive are reconstructed from issued commands using `u_i(z,t)=U_i(t+D_i(z-1))`; this is transport reconstruction for stored commands, not independent reproduction of the paper's 3D rendering.

## Interpretation

The audit distinguishes five questions: whether the original equations were stepped; whether discretization refinement is consistent; whether the graphics match; whether the local linearization is stable; and whether a conditional theorem applies. These are not interchangeable. For this paper, the baselines reproduce closely, but the selected compensated example has a material formula/figure mismatch and an unstable local linearization. No misconduct allegation or author-code attribution is made.
