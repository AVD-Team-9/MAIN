# V-tail geometry, weight and drag model

Added for Concept 5 (HawkTAconcept5, 38.5° V-tail). The model is part of the library, so any requirements file can use it by setting `geometry.tail_type = "v_tail"`. Files without that key behave exactly as before. Concepts 2, 3 and 6 give the same results as before (Concept 6 at the RFP thresholds is still 14,157 lb).

| Piece | Where | JSON keys |
|---|---|---|
| Geometry (sizing the V) | `TfGeom.m`, "V-tail" section | `geometry.tail_type`, `geometry.v_tail` |
| Weight | `TfWeights.v_tail_weight` | `weights.v_tail` |
| Drag: interference | `TfAero.CD0` (header item 7) | `aerodynamics.tail_interference_ratio` |
| Drag: trim | `TfAero.trim_tail_CL`, `trim_drag`, `wing_cm0` (header item 8) | `aerodynamics.trim_drag` |
| Report lines | `Concept_Analysis/concept_sizing.m`, `run_tf_sizing.m` | none |

Notation: S = wing reference area, b = wing span, c̄ = wing MAC, Γ = V-tail dihedral, S_V = total panel area (both panels), b_V = developed span measured along the panels from tip to tip.

---

## 1. Geometry: sizing the V

**Requirement.** As for any tail, the requirement comes from the volume coefficients (Raymer Eqs. 6.28 and 6.29):

```
S_h,req = c_HT S c̄ / L_t          S_v,req = c_VT S b / L_t
```

**Projection rule** (Purser & Campbell, NACA Report 823, 1945; Raymer Sec. 4.5). A panel's normal force is proportional to the flow angle normal to the panel. An angle-of-attack change α gives a normal angle of α cos Γ, and a sideslip β gives β sin Γ. Only the cos Γ part of the force acts in pitch, and only the sin Γ part acts in yaw. To first order, the V is therefore equivalent to

```
horizontal tail of area  S_V cos²Γ       vertical tail of area  S_V sin²Γ
```

and it must satisfy `S_V cos²Γ ≥ S_h,req` and `S_V sin²Γ ≥ S_v,req`.

**Two sizing modes** (`v_tail.sizing`):

| Mode | Γ | S_V |
|---|---|---|
| `"fixed_dihedral"` (used for Concept 5) | given (`dihedral_deg`, the drawn 38.5°) | `max(S_h,req / cos²Γ, S_v,req / sin²Γ)`: the smallest V at that dihedral that meets both requirements |
| `"projected_area"` | `atan √(S_v,req / S_h,req)` | `S_h,req + S_v,req`: the classic rule, both requirements met exactly |

**Concept 5.** The volume coefficients were backed out of the VSP model (c_HT 0.4695, c_VT 0.0559, arm 10.78 ft). Both modes therefore give the drawn V:
- S_V = 90.98 ft², with projected areas 55.72 ft² (horizontal) and 35.26 ft² (vertical);
- b_V = 14.36 ft, projected span b_V cos Γ = 11.24 ft;
- developed aspect ratio 2.267.

**What the rest of the code sees.**
- `S_ht` and `S_vt` return the V's projected areas.
- `S_ht_required` and `S_vt_required` hold the requirements.
- `b_ht` returns the projected span, `b_vt` one panel's height, and `MAC_ht` and `MAC_vt` the panel MAC.
- Since cos² + sin² = 1 and both projections use the panel t/c, the existing `S_wet_ht + S_wet_vt` adds up to the wetted area of the V counted once (Raymer Eq. 7.12).

**Assumptions.**
- The cos²/sin² projections ignore three effects:
  - the panels' lower lift-curve slope at low aspect ratio;
  - interference between the panels;
  - fuselage sidewash.
- Tunnel results after NACA 823 generally show real V-tails need somewhat more area than this rule. To carry that margin, raise the volume coefficients.
- The pitch and yaw arms are taken as equal (one `arm_ft`).
- In `Turbofan_Sizing`, where the wing is resized, the V scales with the wing at fixed volume coefficients and fixed Γ.

---

## 2. Weight

`weights.v_tail.method`:

**`"unit"`**
```
W_V = K_V · ρ_V · S_V,     ρ_V (default) = ρ_ht cos²Γ + ρ_vt sin²Γ
```
The default ρ_V weights the unit weights of the two surfaces the V replaces (4.0 and 5.3 psf, Raymer Table 15.2 fighter column) by the share of the V's area doing each job. That gives 4.50 psf for Concept 5. With K_V = 1 the result equals the previous split-tail answer (409.7 lb).

**`"raymer_fighter"`** (used for Concept 5)
```
W_V = K_V · k_cal · 3.316 (1 + F_w/b_V)^-2.0 (W_dg N_z / 1000)^0.260 S_V^0.806
```
This is Raymer Eq. 15.2, the fighter/attack horizontal-tail equation, applied to the whole V.

| Symbol | Meaning | Concept 5 value |
|---|---|---|
| S_V | total panel area | 90.98 ft² |
| b_V | developed span | 14.36 ft |
| F_w | fuselage width at the tail root (`geometry.v_tail.fuselage_width_at_tail_ft`) | 1.6 ft, placeholder = tailcone end width |
| W_dg | design gross weight = the W_TO being sized | (loop) |
| N_z | ultimate load factor = limit × 1.5. `limit_load_factor: "wing"` uses the wing's limit (`wing_raymer`, `"auto"`) | 4 / 6 / 7 g limit for the RFP / objective / team files |
| k_cal | calibration factor | 1.0531 |
| K_V | V-tail penalty factor | 1.0 |

**Why this equation.**
- Raymer has no V-tail equation.
- The V's root bending comes mainly from the pitch (manoeuvre) load. The horizontal-tail equation scales with W_dg·N_z; the fighter vertical-tail equation has no N_z term.
- Using the whole V area keeps the economy of scale (S^0.806) of one structure instead of two.
- F_w/b_V uses the developed span because the carry-through structure follows the panels.

**Calibration (k_cal = 1.0531), anchored at the T-45C like the wing.** Take a V with the T-45C model's total tail area at the T-45C's weight and load factor:
- area 54.90 + 23.07 = 77.97 ft², developed AR 2.267, F_w 1.6 ft;
- weight 13,500 lb, limit load factor +7.33 g, ultimate factor 1.5.

The calibration makes that V weigh what the T-45C's two unit-weight tails weigh: 4.0 × 54.90 + 5.3 × 23.07 = 341.9 lb. The raw equation gives 324.6 lb, so k_cal = 341.9 / 324.6 = 1.0531.

The empty-weight calibration (`all_else_empty_fraction` 0.2336) was made with those unit-weight tails, so it still holds. Only the trends of the equation (size, W_dg, N_z) change the answer. To reproduce k_cal: build `tf_disciplines('t45c_requirements.json')`, set `geom.S_ref = 179.64`, read `S_ht` and `S_vt`, and apply the two expressions above.

**K_V (`penalty_factor`).** This would cover the V's root fitting, which carries combined pitch, yaw and torsion loads, and the ruddervator mixing hardware. I could not find a published statistical value to cite, so it is left at 1.0. If you use a value from engineering judgement, such as 1.05–1.15, say so in the report.

**Reporting.** `OEW_breakdown` returns `bd.v_tail`. For older tables, it also splits that weight into `horizontal_tail` (cos²Γ share) and `vertical_tail` (sin²Γ share), so totals still add up.

---

## 3. Drag

### 3a. Parasite and interference (`aerodynamics.tail_interference_ratio`, r)
```
CD0 = Cfe S_wet / S  +  (r − 1) Cfe (S_wet,ht + S_wet,vt) / S
```
Cfe is an average that already includes typical interference. Raymer Sec. 12.5 gives interference factors of Q ≈ 1.03 for a V-tail (two root junctions) against 1.04–1.05 for a conventional tail (three junctions). r = 1.03/1.045 ≈ 0.986 would therefore convert a conventional-tail Cfe to a V-tail.

**Concept 5 uses r = 1.0.** Its Cfe comes from the VSPAERO parasite-drag build-up of this same V-tailed model, so the V is already counted.

### 3b. Trim drag (`aerodynamics.trim_drag`) — the V's main aerodynamic penalty

This term is added to `TfAero.CD` in the clean configuration only. That covers every cruise, climb, turn, Ps and ceiling calculation, but not takeoff or landing.

**Wing zero-lift moment** (Raymer Ch. 16, the finite-wing and sweep correction of the section value; the twist term is omitted):
```
Cm0_w = cm0_airfoil · A cos²Λ / (A + 2 cos Λ)
```

**Tail load to trim.** Moment balance about the CG (nose-up positive, tail lift positive up):
```
0 = Cm0_w + CL·Δh − CL_h · L_t / c̄      ⇒      CL_h = (Cm0_w + CL·Δh) · c̄ / L_t
```
Here Δh = (x_cg − x_ac,w)/c̄ and CL_h = L_tail / (q S). For a V, L_tail is the vertical resultant of the two panels' normal forces.

**Drag increment:**
```
ΔCD_trim = K [ (CL − CL_h)² − CL² ]  +  CL_h² · S / (π e_t b_t²)
```
- The first term: the wing carries CL − CL_h, so a tail download costs extra wing induced drag.
- The second term: the tail's own induced drag, D_i = L_h² / (q π e_t b_t²).

**Span used for a V-tail.** The total panel normal force is N = L_h / cos Γ, and each panel's induced drag follows its developed span:
```
D_i = N² / (q π e_t b_V²) = L_h² / (q π e_t (b_V cos Γ)²)
```
This matches Munk's projected-span result. `geom.trim_tail_span` is therefore b_V cos Γ (11.24 ft for Concept 5) instead of the developed 14.36 ft. A V-tail pays more trim drag than a flat tail of the same developed span.

**Concept 5 inputs and results:**

| Input | Value | Status |
|---|---|---|
| cm0_airfoil | −0.04 | Approximate value for the NACA 64A2xx sections. The inboard 64A010 has 0. Check it against Abbott & von Doenhoff or XFOIL. |
| cg_minus_wing_ac_over_mac (Δh) | 0.0 | Placeholder: CG at the wing aerodynamic centre. Set it from the CG estimate. |
| tail_span_efficiency (e_t) | 0.8 | Placeholder |

These give Cm0_w = −0.0276 and CL_h = −0.016 (a download). The trim drag is ΔCD = 0.00082 at CL 0.3 and 0.00230 at CL 1.0, about 5 % of CD0 in cruise.

**Assumptions.**
- Only the wing moment and the wing-lift moment are balanced. Fuselage and nacelle moments, the thrust-line offset and the tail's own Cm0 are neglected.
- The tail arm L_t (wing c̄/4 to tail c̄/4) is used as the CG-to-tail distance.
- There is no CG travel with fuel burn.
- Downwash tilt of the tail lift, the tail's profile-drag rise with lift, and the small non-planar span-efficiency gain of a V are neglected; the last of these is conservative.
- Flap pitching moments are not modelled, so trim is not applied with flaps down.
- Wave drag on the tail (t/c 0.06, 34.7° sweep) is neglected.

> **All four concepts now carry `trim_drag`** with the same placeholder inputs, so they are compared on the same basis (Concept 2 twin fins: [TWIN_TAIL_MODEL.md](TWIN_TAIL_MODEL.md)). At CL 1.0 the trim drag is 21.4 (C3), 23.0 (C5), 23.4 (C6) and 27.0 (C2) counts.

---

## 4. Effect on Concept 5 (Concept_Analysis, fixed geometry, F124)

| File | W_TO before → after | V-tail weight before → after | Worst margin before → after |
|---|---|---|---|
| `concept5_rfp` (4 g) | 14,004 → 13,911 lb | 409.7 → 338.7 lb | Sustained 4 g −1.9 % → −3.5 % |
| `concept5_objective` (6 g) | 14,401 → 14,383 lb | 409.7 → 379.6 lb | M0.90 −51.6 % → −51.9 %; 6 g −42.2 % → −43.3 % |
| `concept5_team` (7 g) | 14,577 → 14,592 lb | 409.7 → 396.6 lb | M0.90 −51.7 % → −52.0 %; 4 g −5.9 % → −8.2 % |

- **Weight.** The V comes out lighter than the old unit-weight split at the threshold load factor, because the old 4.0/5.3 psf numbers were tied to a 7.33 g airplane. The gap narrows at 6 and 7 g.
- **Drag.** Trim drag costs about 1–2.5 points of margin on the turn and Ps requirements, and 4–6 points on the 35k cruise and 45k ceiling.
- **Regression check.** Conventional-tail concepts give the same results as before (Concept 6 at the RFP thresholds: 14,157 lb).
