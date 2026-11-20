# Twin canted vertical tail: geometry, weight and drag model

This model was added for Concept 2 (HawkTAconcept2). Concept 2 has all-moving stabilators and two vertical fins canted 15° outboard. It is the twin-fin counterpart of [V_TAIL_MODEL.md](V_TAIL_MODEL.md).

Any requirements file can use it by setting `geometry.tail_type = "twin_vertical"`. Files without that key behave as before. After this change Concept 5 still gives 13,911 lb and Concept 6 still gives 14,157 lb (RFP thresholds).

| Piece | Where | JSON keys |
|---|---|---|
| Geometry: fin projection and sizing | `TfGeom.m`, "Twin vertical tails" section | `geometry.tail_type`, extra keys in `geometry.vertical_tail` |
| Weight | `TfWeights.fin_weight`, `TfWeights.raymer_fighter_vt` | `weights.twin_vertical` |
| Drag: interference | `TfAero.CD0` (header item 7, shared with the V-tail) | `aerodynamics.tail_interference_ratio` |
| Drag: trim | `TfAero.trim_drag` (header item 8, shared) | `aerodynamics.trim_drag` |
| Report line | `Concept_Analysis/concept_sizing.m` ("fins") | — |

Notation:
- φ = fin cant from vertical, outboard positive.
- n = number of fins.
- S_vt = true (panel) area of all fins; S_fin = S_vt / n.
- S, b, c̄ = wing area, span and MAC.

---

## 1. Geometry: projection and sizing

**Projection.** A fin canted φ from vertical sees a sideslip β as a normal flow angle β cos φ, and only cos φ of its normal force acts sideways. The same applies to an angle-of-attack change with sin φ. This is the V-tail projection argument (Purser & Campbell, NACA Report 823) with Γ = 90° − φ:

```
yaw-effective (vertical) area  = S_vt cos²φ
horizontal (pitch) projection  = S_vt sin²φ
```

**Sizing.** The volume coefficient is now the effective (yaw) coefficient, based on the vertical projection:

```
S_v,req = c_VT,eff S b / L_vt      (Raymer Eq. 6.28)
S_vt    = S_v,req / cos²φ          (true area of all fins)
S_fin   = S_vt / n,   fin span h = √(AR_fin S_fin) along the fin,   MAC from S_fin, h, λ
```

The wetted area (Raymer Eq. 7.12) and the weight use the true area S_vt.

The horizontal-tail requirement is unchanged (Raymer Eq. 6.29). With `credit_fins_in_pitch = true` it becomes `S_ht = max(S_h,req − S_vt sin²φ, 0)`, but by default the fins' pitch projection is not credited.

**New `geometry.vertical_tail` keys:**

| Key | Default | Meaning |
|---|---|---|
| `n_fins` | 2 | number of fins |
| `cant_deg` | 0 | outboard cant from vertical (0–60°) |
| `credit_fins_in_pitch` | false | count the fins' horizontal projection toward the HT requirement |
| `volume_coefficient` | — | effective c_VT on the vertical projection |
| `AR`, `taper_ratio`, ... | — | one fin's (AR = h²/S_fin) |

**Concept 2, from the VSP component `TwinVerticalTails`:**
- 2 fins, X-rotation 75° (cant 15°), at y = ±1.0 ft.
- 28.32 ft² total (14.16 ft² per fin), span 4.09 ft per fin measured along the fin, AR 1.18 per fin.

The old file treated the whole 28.32 ft² as vertical tail, which overstates directional effectiveness by 1/cos²15° = 7 %. The effective coefficient is now 0.08773 × cos²15° = 0.08185. With it, the code rebuilds the drawn fins: 28.32 ft² true area, 26.42 ft² yaw-effective. Their horizontal projection (1.90 ft²) is not credited.

**Assumptions.**
- No fin-to-fin or fin-fuselage end-plate interference. Fins 2 ft apart on the fuselage, as on Concept 2, can gain or lose effectiveness from it.
- The main reason twin fins are chosen, directional stability at high angle of attack (fins outside the forebody wake), is not credited. This is a cruise/manoeuvre sizing model, not a high-α one.
- The cos² projection ignores each fin's lift-curve slope at low aspect ratio (as for the V-tail).

---

## 2. Weight

`weights.twin_vertical.method`:

### `"unit"` (default without the block)

```
W_fins = K_F ρ_vt S_vt
```

ρ_vt is 5.3 psf (Raymer Table 15.2), applied to the true fin area.

### `"raymer_fighter"` (used for Concept 2)

Raymer Eq. 15.3, the fighter/attack vertical tail, for one surface:

```
W_1(S) = 0.452 K_rht (1 + H_t/H_v)^0.5 (W_dg N_z)^0.488 S^0.718 M^0.341 L_t^-1.0
         (1 + S_r/S)^0.348 A^0.223 (1 + λ)^0.25 (cos Λ_vt)^-0.323

fin_area_basis "per_fin" (used):   W_fins = K_F k n W_1(S_vt / n)
fin_area_basis "total":            W_fins = K_F k   W_1(S_vt)
```

| Symbol | Meaning | Concept 2 |
|---|---|---|
| S | fin true area (per fin or total) | 14.16 / 28.32 ft² |
| A, λ, Λ_vt | one fin's aspect ratio, taper, quarter-chord sweep | 1.18, 0.515, 38.2° |
| L_t | tail arm | 11.72 ft |
| W_dg | design gross weight (the W_TO being sized) | (loop) |
| N_z | ultimate = limit × 1.5; `"wing"` = the wing's limit (4 / 6 / 7 g for RFP / objective / team) | |
| M | design maximum Mach (`max_mach`) | 0.8, placeholder |
| S_r/S | rudder area fraction | 0.25, placeholder |
| K_rht | 1.047 for a rolling tail, else 1 | 1.0 |
| H_t/H_v | 0 for a tail mounted on the fuselage, 1 for a T-tail | 0 |
| k | calibration factor | 0.9668 |
| K_F | penalty factor | 1.0 |

**Per fin vs. total.** S enters as S^0.718, so two fins of half the area weigh 2^(1−0.718) = 1.22 times one fin of the full area. That is the structural cost of splitting the fin: two roots, two attachments, two actuators. "per_fin" is applied here, treating each fin as its own surface; "total" gives the single-fin answer for comparison. A real design lies between the two, depending on how much structure the fins share.

**Calibration (k = 0.9668), anchored at the T-45C, as for the wing and the V-tail.** Take the T-45C model's single fin: 23.07 ft², AR 1.3, taper 0.4, 35° sweep, arm 14.0 ft. At 13,500 lb and +7.33 g × 1.5, the equation gives 126.5 lb. The unit weight gives 5.3 × 23.07 = 122.3 lb, so k = 122.3 / 126.5 = 0.9668.

The empty-weight calibration was made with that unit weight, so it still holds. M, S_r/S, K_rht and H_t/H_v are the same at the anchor and on Concept 2, so they cancel unless you change them for the concept. The T-45C tail areas are themselves placeholders (Raymer Table 6.4 volume coefficients).

To reproduce k: build `tf_disciplines('t45c_requirements.json')`, set `geom.S_ref = 179.64`, and call `TfWeights.raymer_fighter_vt(13500, 7.33*1.5, geom.S_vt, 0.8, geom.L_vt, 0.25, geom.AR_vt, geom.taper_vt, geom.sweep_qc_vt_deg, 1, 0)`.

**The stabilators stay on the unit weight** (4.0 psf), as before.

**Reporting.** The fins go in `bd.vertical_tail`. No new row is added, so all tables are unchanged.

---

## 3. Drag

**Parasite.** The wetted area follows the true fin area, so splitting the same area into two fins adds no wetted area.

The real twin-fin parasite penalties are:
- two extra junctions;
- smaller chords, so a lower Reynolds number and a higher skin-friction coefficient.

Both are already in Concept 2's Cfe, because it comes from the VSPAERO parasite-drag build-up of this twin-fin model. `tail_interference_ratio` is therefore 1.0. Raymer Sec. 12.5 gives no twin-fin interference factor (conventional tail Q ≈ 1.04–1.05, H-tail 1.08). If you reuse this model with a Cfe from a single-fin airplane, raise r to cover it.

**Trim drag.** This is the same model as for the V-tail ([V_TAIL_MODEL.md §3b](V_TAIL_MODEL.md)), with the stabilator span as the span that carries the trim load:

```
CL_h = (Cm0_w + CL Δh) c̄ / L_t,      Cm0_w = cm0 A cos²Λ / (A + 2 cos Λ)
ΔCD  = K [ (CL − CL_h)² − CL² ] + CL_h² S / (π e_t b_h²),    b_h = 12.40 ft (VSP)
```

| Input | Concept 2 value | Status |
|---|---|---|
| cm0_airfoil | −0.04 | approximate for the 64A208/64A206 sections |
| Δh (CG minus wing ac) | 0.0 | placeholder, same as Concept 5 |
| e_t | 0.8 | placeholder, same as Concept 5 |

These give Cm0_w = −0.0214 and CL_h = −0.013. The trim drag is ΔCD = 0.00087 at CL 0.3 and 0.00270 at CL 1.0. That is larger than Concept 5's at CL 1.0, because Concept 2's low-aspect-ratio wing has a larger K.

---

## 4. Effect on Concept 2 (Concept_Analysis, fixed geometry, F124)

| File | W_TO before → after | Fin weight before → after | Notable margins before → after |
|---|---|---|---|
| `concept2_rfp` (4 g) | 14,200 → 14,258 lb | 150.1 → 159.0 lb | takeoff −20.3 → −20.9 %; sustained 4 g +11.1 → +7.3 % |
| `concept2_objective` (6 g) | 14,533 → 14,666 lb | 150.1 → 196.5 lb | takeoff −23.9 → −25.3 %; 6 g −44.3 → −46.4 % |
| `concept2_team` (7 g) | 14,680 → 14,848 lb | 150.1 → 213.2 lb | +7 g inst. −3.8 → −4.9 %; 45k ceiling −3.7 → −7.9 % |

How the change splits up:
- **Fins (weight only, trim off):** +17 / +90 / +123 lb of W_TO for the RFP / objective / team files. The rest of the change is trim drag.
- **`fin_area_basis = "total"` instead of "per_fin":** fins about 18 % lighter (130.6 / 161.3 / 174.9 lb).
- **Pass/fail:** no result changes from pass to fail at the RFP thresholds. In the team set, the 45k ceiling and the +7 g instantaneous turn already failed and now fail by more.

**All four concepts now carry trim drag** with the same placeholder inputs, so they are compared on the same basis.
