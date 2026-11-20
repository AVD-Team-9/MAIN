# Concept Analysis: fixed geometry, sized takeoff weight

Analyses a **given** concept. Every physical dimension is held fixed: the
wing area, the planform, the tails and the fuselage, plus the engine if it is
a real one. The sizing loop finds the **takeoff weight** the concept needs to
fly its missions. Every requirement is then **checked** at the resulting
design point, and the code finds the heaviest weight at which the concept
still meets them all.

It reads the **same JSON format** as `../Turbofan_Sizing` plus one key,
`geometry.S_ref_fixed_ft2`, and reuses that folder's classes and functions.
No code is copied, so a fix in Turbofan_Sizing applies here too.

## How to run

```matlab
cd Concept_Analysis                 % Turbofan_Sizing is added to the path automatically

r = concept_sizing('concept5_rfp.json');                      % one concept, full report + diagram
r = concept_sizing('concept5_rfp.json', 'save_png', true);    % also writes output/concept5_rfp_concept_diagram.png

% several concepts side by side (one tile each, own curves)
concept_compare({'concept2_rfp.json', 'concept3_rfp.json', 'concept5_rfp.json', 'concept6_rfp.json'}, ...
                'title', 'RFP thresholds', 'save_png', true);
```

Files in `inputs/` are found by name. Do **not** put the parent `IHW3_Code`
(TTPA) folder on the path; `concept_setup` stops with instructions if it is
there.

## Design vs. analysis

| | `Turbofan_Sizing/run_tf_sizing` (design) | `concept_sizing` (analysis) |
|---|---|---|
| Wing area | chosen every pass (smallest wing that works) | **fixed**: `geometry.S_ref_fixed_ft2` |
| Span, chords, tails | follow the chosen wing | fixed (they follow the fixed wing) |
| Engine | rubber (sized) or fixed | fixed engine: as is. Rubber engine: least thrust meeting every requirement **at the concept's W/S** |
| W_TO | closed by the loop | closed by the loop |
| Requirements | design point put **on** the feasible boundary | **checked** at the design point, with a margin each; failures are reported, not errors |
| Extra output | — | heaviest W_TO that still meets every requirement (weight-growth margin) |

## How it works

1. **Airplane:** S_ref is fixed. Thrust is the engine's (fixed), or for a rubber engine the least thrust that meets the requirements at W_0/S_ref.
2. **Missions:** every mission in the file is flown at W_0, and the hungriest sets the fuel.
3. **Empty weight:** the component build-up at W_0. Wing, tails and fuselage are now constant.
4. **Closure:** solve f(W_0) = W_0 − payload − fuel − OEW = 0. Only W_0 is unknown, so this is a bracketed root-find (Illinois regula falsi). It converges in about 4–6 evaluations with no relaxation factor. Under-relaxation, as the design loop uses, overshoots here, because with the wing fixed the closure becomes very steep.
5. **Check:** every requirement is evaluated at W/S = W_0/S_ref and T/W = T_SL/W_0 (`concept_constraint_check`):
   - thrust requirement: margin = available / needed − 1;
   - wing-loading wall (landing, approach, instantaneous g): margin = wall / (W/S) − 1.
6. **Growth margin:** with S_ref and T_SL fixed, a heavier airplane moves along T/W = T_SL / (S_ref · W/S), right and down on the diagram, until it hits the first requirement. That weight is the **heaviest feasible W_TO**.

On the diagram the diamond is the converged design and the dotted line is the weight-growth path. The × marks the heaviest feasible W_TO. If the × is **left of** the diamond, the concept is already too heavy for its wing and engine.

## Files

| File | Purpose |
|---|---|
| `concept_sizing.m` | driver: closure, requirement check, report, diagram |
| `concept_constraint_check.m` | margins of every requirement at a weight, and the heaviest feasible weight |
| `concept_compare.m` | several concepts: table plus a grid of diagrams (one tile each) |
| `concept_setup.m` | adds `../Turbofan_Sizing` to the path, resolves input files |
| `inputs/conceptN_rfp.json` | Concept N (VSPAERO aero, fixed S_ref) against the RFP thresholds only |
| `inputs/conceptN_objective.json` | Concept N against the RFP objectives (M 0.90, Ps 125 ft/s, sustained 6.0 g; field requirements at threshold) |
| `inputs/conceptN_team.json` | Concept N against the team requirements (thresholds + M 0.90, 35,000 ft cruise, 45,000 ft ceiling, ±g instantaneous) |
| `output/` | figures |

The Concept 5 files model its V-tail directly (`geometry.tail_type = "v_tail"`):
- the V is sized from the volume coefficients at the drawn 38.5° dihedral;
- its weight comes from Raymer's fighter tail equation, anchored at the T-45C;
- trim drag uses the V's projected span.

The report prints a V-tail line and a trim line. See
[../Turbofan_Sizing/V_TAIL_MODEL.md](../Turbofan_Sizing/V_TAIL_MODEL.md).
The Concept 2 files model its twin fins canted 15° (`geometry.tail_type = "twin_vertical"`):
- the fins are sized on their yaw-effective projection;
- their weight comes from Raymer's fighter vertical-tail equation, applied to each fin and anchored at the T-45C;
- trim drag uses the stabilator span.

The report prints a fins line. See
[../Turbofan_Sizing/TWIN_TAIL_MODEL.md](../Turbofan_Sizing/TWIN_TAIL_MODEL.md).
All four concepts include trim drag with the same placeholder inputs.

Any Turbofan_Sizing requirements file works once you add
`"S_ref_fixed_ft2": <area>` to its `geometry` block, or pass `'S_ref', <area>`.

## Results for the AVD CoDR concepts (F124-GA-100, both RFP missions)

The concept inputs use the **OpenVSP models** (`../Turbofan_Sizing/AVD_CoDR_Aero_data/VSP/*.vsp3`) for:
- **Wing:** area, AR, equivalent taper, NACA 64A thickness ratios, and the quarter-chord sweep, **corrected** because the VSP sweeps are leading-edge sweeps.
- **Tails:** areas and moment arms, entered as volume coefficients that reproduce the VSP areas exactly.

The VSPAERO aero reports supply Cfe, CLmax (recomputed with the corrected sweep) and the flap increments. **Wing weight** uses Raymer's fighter wing equation (`weights.wing_weight_method = "raymer_fighter"`, calibrated at the T-45C; the limit load factor is `"auto"`, set by each file's requirements: 4.0 g thresholds, 6.0 g objectives, 7.0 g team), so aspect ratio, sweep, thickness and load factor carry their structural weight.

**RFP thresholds** (`concept_comparison_rfp.png`):

| Concept | S_ref (fixed) | Tails (VSP) | W_TO | Result | Heaviest feasible W_TO |
|---|---|---|---|---|---|
| 2 | 160.3 ft², AR 3.47 | stabilators 58.1 + twin fins 28.3 ft² (15° cant) | 14,258 lb | **fails takeoff** (−20.9 %) and critical field (−5.1 %) | 12,677 lb (−11.1 %) |
| 3 | 190.1 ft² | HT 56.2 + VT 29.5 + ventral 1.1 ft² | 14,000 lb | **fails takeoff by −0.4 %** (was exactly at the limit before trim drag) | 13,971 lb (−0.2 %) |
| 5 | 204.1 ft² | V-tail 91.0 ft² (38.5°) | 13,911 lb | **fails sustained 4 g** (−3.5 %; ref. M 0.80 is above its M_dd) | 13,415 lb (−3.6 %) |
| 6 | 215.5 ft² | HT 74.4 + VT 29.5 ft² | 14,192 lb | all met; tightest is sustained 4 g +12.8 % | 15,123 lb (+6.6 %) |

All four include **trim drag** (`aerodynamics.trim_drag`, same placeholder cm0 −0.04, CG at the wing a.c., e_t 0.8). Concept 5 has the V-tail weight model and Concept 2 the twin-fin model (see the notes above). Trim drag costs 3–4 points of sustained-turn margin (C5 loses only 1.6, because its lighter V-tail offsets part of it) and about 6 points on the M0.78 max speed.

Weights: Raymer wing equation anchored at the T-45C, all-else fraction recalibrated (0.2336) so the build-up reproduces the L-39C exactly and the T-45C within +0.8 %. The wing structure follows each file's requirements: 4.0 g here.

**RFP objectives** (`concept_comparison_objective.png`, `inputs/conceptN_objective.json`, structure 6.0 g): no concept meets them as drawn (with trim drag: C2 14,666 lb, 3 failures; C3 14,381 lb, 3; C5 14,383 lb, 2; C6 14,619 lb, 2). The improvement packages below were found BEFORE the trim-drag, V-tail and twin-fin models were added, so their 6 g margins are about 2 points optimistic. The lightest packages that meet all three objectives (M 0.90, Ps 125 ft/s, sustained 6.0 g) are:
- Concept 6: supercritical section + AR +1.0, 14,740 lb, 6 g margin +4 %;
- Concepts 3 and 5: supercritical section + 25 % more wing + AR +1.5, about 15,300–15,400 lb, +3 %;
- Concept 2: none.

**Team requirements** (`concept_comparison_team.png`, structure 7.0 g): every concept fails the
M 0.90 sustained speed (−25 % to −52 %): with conventional 64A sections at these
sweeps the drag-divergence Mach is only 0.77–0.86.

Caveats:
- **Concept 2's aerodynamics come from its own (updated) VSPAERO run.** The code's CD0 is within 1.6 % of it. That report, like the others, still quotes the leading-edge sweep as quarter-chord, so the files keep the corrected sweep and CLmax.
- **Side-of-body station** for the root t/c is taken at 2.25 ft.
- Inherited from `../Turbofan_Sizing/README.md`:
  - thrust lapse and TSFC are generic Mattingly models;
  - the 30-minute military-thrust maneuvering block dominates the fuel.
