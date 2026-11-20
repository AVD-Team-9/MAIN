# Turbofan Sizing — constraint diagram and engine sizing

A turbofan version of the IHW3 TTPA sizing code. It produces a **T/W – W/S
constraint diagram** for a twin-turbofan airplane and **sizes the engines**,
either at a takeoff weight you give it or by closing weight, wing and thrust
together in the sizing loop.

> **Every number in `tf_requirements.json` is a placeholder** for a generic
> light business jet (2 crew + 6 passengers, 1,800 nmi at M 0.75 / 41,000 ft).
> Replace the requirements and the model inputs with your aircraft's before
> using any result. The JSON says where each placeholder came from.

> **Fixed-geometry concept analysis** (wing area held fixed, W_TO sized,
> requirements checked) lives in `../Concept_Analysis`. It reads the same
> JSON format and reuses this folder's code.

## How to run

Run from **this folder**. Do **not** put this folder and the parent
`IHW3_Code` folder on the MATLAB path at the same time: several file names
exist in both (`get_state`, `get_con`, `run_constraints`, `matching_envelope`,
`sizing_pass`, …) and MATLAB would mix the two versions.

```matlab
cd Turbofan_Sizing

% 1. Quick engine size at a takeoff weight you already have
R = run_tf_constraints('W_TO', 16000, 'design_point', [62 0.47]);

% 2. Full sizing: weight, wing and engine all closed together
[result, hist] = run_tf_sizing();
result.engine          % the engine size
```

Both accept `'save_png', true` to write their figures to `output/`, and
`'json', 'other_file.json'` to run a different aircraft. PNG names start with
the JSON name minus `_requirements` (`tf_*.png`, `t45c_*.png`), so two aircraft
never overwrite each other's figures.

```matlab
% T-45C validation case (single engine, carrier trainer) - see below
R = run_tf_constraints('json', 't45c_requirements.json');
[result, hist] = run_tf_sizing('json', 't45c_requirements.json');
```

### What each driver gives you

| | `run_tf_constraints` | `run_tf_sizing` |
|---|---|---|
| Takeoff weight | **given** (`W_TO`) | **computed** (the loop closes it) |
| Constraint diagram | yes, at the given weight | yes, of the converged airplane |
| Engine thrust | `T/W × W_TO`, at the corner and at your design point | converged `T_SL` |
| Engine weight and size | Raymer Eqs. 10.1–10.3 | same, and fed back into OEW and drag |
| Mission fuel, OEW | no | yes |
| Use it when | you have a W_TO estimate and want a thrust class fast | you want a self-consistent engine and airplane |

Thrust is reported both **installed** (what the constraints need) and
**uninstalled per engine** (installed ÷ (1 − installation loss)). Compare the
uninstalled figure with engine catalogues.

### Result with the placeholder inputs

`run_tf_sizing` closes in 40 passes and reaches the same answer from guesses
of 10,000, 16,000 and 25,000 lbf:

```
W_TO 12,938 lbf    S_ref 198.9 ft^2    W/S 65.05 psf (on the landing wall)
T_SL 6,144 lbf installed (T/W 0.475), driven by the initial cruise ceiling
about 3,230 lbf uninstalled thrust per engine
```

That is the class of the existing light twin business jets, which suggests
the models hang together. It is **not** validated against any particular
airplane.

---

## T-45C Goshawk validation case (`t45c_requirements.json`)

A real airplane to test the models against: single engine (F405-RR-401,
5,527 lbf), buried in the fuselage, carrier-capable, 13,500 lb MTOW,
179.64 ft² wing, 9,394 lb empty. Every input in the file is tagged with its
source: US Navy fact file, Navy T-45 aerodynamics workbook (CNATRA P-1231,
Appendix A), secondary engine data, or **PLACEHOLDER**. CLmax, drag, tail sizes,
approach speed and field length are not public; they are in the
distribution-limited NATOPS manual.

What is calibrated:

| Item | How | Value |
|---|---|---|
| Engine weight, length, diameter | Raymer 10.1–10.3 coefficients scaled to the Adour Mk 871 (1,299 lb, 6.43 ft, 1.87 ft) | 0.1028, 0.2115, 0.0244 |
| TSFC | `tsfc_scale` = 0.81 / 0.90 (Adour SLS ÷ Mattingly low-bypass) | 0.90 |
| Empty weight | Raymer fighter column, Navy gear (0.045), then all-else fraction set to give 9,394 lb at the real size | 0.228 (Raymer fighter: 0.17) |

The requirements are not FAR 25. They are field takeoff, carrier approach
speed (a new `approach_speed` wall), wave-off climb (all engines), and the
fact-file maximum speed (560 kt at sea level) and ceiling (42,500 ft) used
as requirements.

### Results and what they say

**Fixed weight, 13,500 lb** (`run_tf_constraints`): the real airplane (75.2 psf,
T/W 0.389 installed) is feasible against every requirement. Sea-level
maximum speed drives the least-thrust corner, which needs only **4,494 lbf
uninstalled, 19 % below the real 5,527 lbf**. So either the real engine has
margin these requirements don't capture (catapult, hot day, acceleration,
spool-up on wave-off, or the max speed is at altitude), or the model is
optimistic. The likeliest model term is the low-bypass lapse at M 0.84 on
the deck, which is sensitive to the placeholder `throttle_ratio` = 1.05.

**Closed loop** (`run_tf_sizing`): 10,631 lb, 148.9 ft², 3,751 lbf
uninstalled, 1,998 lb fuel. The loop builds a **smaller** airplane than the
real one, because the 700 nmi ferry mission needs only about 2,000 lb of the
2,904 lb the T-45C carries. Training sorties, reserves and endurance are what
fill the real tanks. Because the calibrated all-else fraction scales with
W_TO, the empty weight shrinks with it.

In short, the calibrated weights reproduce the real airplane **at its real
size**. The requirements and the mission, which are placeholders, are what
move the closed design. Replace them with NATOPS or course data before
drawing conclusions. Wing fuel volume is about 14 % short, as expected,
because the T-45 carries most of its fuel in the fuselage.

---

## What changed from the TTPA code, and why

| Area | TTPA (piston + propeller) | Turbofan | Files |
|---|---|---|---|
| Engine size state | `P_SL` [hp] | `T_SL` [lbf], installed, sea-level static, takeoff rating | `TfProp` |
| Lapse | `1.132σ − 0.132` (density only) | Mattingly Eq. 2.54a (high bypass) or 2.54b (low bypass, dry), in θ₀, δ₀, M and throttle ratio | `TfProp.thrust_lapse` |
| Fuel consumption | BSFC [lbm/(hp·hr)], constant | TSFC from Mattingly Eq. 3.55a (high bypass) or 3.55b (low bypass, dry) [1/hr] | `TfProp.tsfc` |
| Propeller efficiency | η_p per segment | none | — |
| Engine-out factor | 1/n | (n − 1)/n, correct for any engine count | `TfProp.thrust_ratio` |
| Engine weight/size | Raymer Table 10.4 piston fits; fixed nacelle diameter | Raymer Eqs. 10.1–10.3 jet rubber engine; nacelle **diameter** follows thrust too | `TfProp`, `TfGeom` |
| Atmosphere | ρ, σ only | + θ₀, δ₀, T, P, a, V, q; cache keyed on (altitude, **Mach**) | `get_state` |
| Diagram axis | W/P, feasible **below** the curves | T/W, feasible **above** the curves | all constraints |
| Envelope | min of curves; best = max W/P | **max** of curves; best = **min** T/W | `matching_envelope` |
| Takeoff | FAR 23 ground roll, TOP23 | FAR 25 field length, TOFL = 37.5·TOP25 (Roskam I §3.4) | `constraint_takeoff_far25` |
| Landing | FAR 23 ground roll, 0.265·Vs² | FAR 25 field length, 0.3·V_A², V_A = 1.3 Vs | `constraint_landing_far25` |
| Climb | Roskam CGRP (propeller form), CL = CLmax − 0.2 | T/W = (β/k_T)(G + 1/(L/D)), CL = CLmax/k² at the FAR 25 speeds | `constraint_climb_far25` |
| Cruise | power index / drag-based power balance | Mattingly master equation (cruise, ceiling, Ps, turns) | `constraint_flight` |
| Oswald e | Raymer 12.48 (straight wing) | 12.48 or 12.49 by leading-edge sweep | `TfAero.e` |
| Compressibility | none | Korn drag divergence + Lock's 4th-power wave drag; κ from `airfoil_type` | `TfAero.CD`, `wave_drag` |
| OEI drag | stopped propeller | windmilling engine | `TfAero.get_config_polar` |
| Weights | Raymer 15.2 GA column | between GA and transport (**placeholders, calibrate**) | JSON `weights` |
| Mission | propeller Breguet; fixed fractions hardcoded | jet Breguet range and endurance; climb on excess thrust with an EAS/Mach schedule; fixed fractions **read** from the JSON | `segment_*_jet`, `run_mission_jet` |
| Sizing step 2 | P = W₀ / (W/P) | T = W₀ × (T/W) | `sizing_pass` |
| Plot | none | `plot_constraint_diagram` | new |

## Files

**New or rewritten** (each has a detailed header comment explaining the
equations, sources and what changed from the TTPA version):

| File | Purpose |
|---|---|
| `tf_requirements.json` | every requirement and model input, with a source note for each |
| `tf_requirements_path.m` | locates the JSON |
| `ThrustPropulsionBase.m` | abstract thrust-engine interface (replaces `PropulsionBase2`) |
| `TfProp.m` | thrust lapse, TSFC, engine weight/length/diameter |
| `TfGeom.m` | wing, tails, fuselage, thrust-scaled nacelles, wetted areas |
| `TfAero.m` | CD0 from geometry, swept-wing e, wave drag, configuration polars |
| `TfWeights.m` | component weight build-up |
| `tf_disciplines.m` | builds the `obj` bundle |
| `get_state.m`, `get_con.m`, `get_miss_seg.m` | atmosphere with Mach; jet condition and segment keys |
| `constraint_takeoff_far25.m`, `constraint_landing_far25.m`, `constraint_climb_far25.m`, `constraint_flight.m` | the FAR 25 constraints |
| `tf_json_with_overrides.m` | runs a modified copy of a JSON with a different airfoil type and/or sweep |
| `compare_engine_points.m` | one constraint diagram with the design point of several engines (each its own sized airplane, or all at a common W_TO) |
| `compare_concepts.m` | grid of constraint diagrams, one tile per concept, each with its own curves and sized design point |
| `airfoil_sweep_study.m` | sizes the airplane for every airfoil type × sweep and plots the trade |
| `fixed_engine_wing_loading.m` | with a fixed engine: the smallest wing (largest W/S) the engine's T/W allows |
| `segment_fixed_thrust_jet.m` | mission segment: fixed time at military, max-continuous, drag-matched or idle (fuel-flow) thrust |
| `constraint_instantaneous_turn.m` | instantaneous (lift-limited) ±g -> W/S wall; negative g uses `aerodynamics.CLmin_clean` |
| `constraint_approach_speed.m` | maximum approach speed -> W/S wall (carrier aircraft) |
| `t45c_requirements.json` | T-45C Goshawk validation case, with a source for every value |
| `prm2_requirements.json`, `prm2_objective_requirements.json` | PRM2 naval trainer RFP (from `PRM2_requirements.slreqx`), threshold and objective; each condition cites its requirement ID |
| `prm2_f124_requirements.json` | PRM2 team case sized around the **F124-GA-100** (fixed engine), flying **both** RFP missions (training and ferry), weights calibrated to the L-39C |
| `prm2_concept2/3/5/6_requirements.json` | the F124 case with each AVD CoDR concept's VSPAERO aerodynamics and wing planform (from `AVD_CoDR_Aero_data/`) |
| `prm2_concept2/3/5/6_objective_requirements.json` | the same concepts against the RFP objectives (M 0.90, Ps 125 ft/s, sustained 6.0 g) |
| `prm2_concept2/3/5/6_rfp_requirements.json` | the same concepts, unmodified, against the RFP threshold requirements only |
| `prm2_f404_requirements.json` | same as the F124 case with the **F404-GE-100D** (dry, 11,000 lbf) instead, for the engine trade |
| `prm2_7g_requirements.json` | PRM2 team case: M 0.90 sustained speed, 35,000 ft cruise, 45,000 ft service ceiling, +7/−3 g instantaneous at 15,000 ft / M 0.70 (plus the RFP thresholds) |
| `prm2_objective_ceiling_requirements.json` | PRM2 objective requirements (M 0.90, Ps 125, 6 g) with a 35,000 ft cruise capability and a 45,000 ft service ceiling |
| `prm2_m090_requirements.json` | PRM2 threshold case with the M 0.90 max-speed objective (Ps and sustained g at threshold) |
| `prm2_45k_requirements.json` | PRM2 threshold case with the ferry cruise at 45,000 ft (adds a 300 ft/min ceiling constraint at 45,000 ft) |
| `run_constraints.m`, `matching_envelope.m`, `wing_loading_sweep.m` | constraint analysis (envelope flipped) |
| `plot_constraint_diagram.m` | draws the diagram |
| `segment_climb_jet.m`, `segment_cruise_jet.m`, `segment_loiter_jet.m`, `run_mission_jet.m`, `mission_fuel.m` | jet mission |
| `sizing_pass.m`, `sizing_mainloop.m` | the loop (thrust instead of power) |
| `run_tf_constraints.m`, `run_tf_sizing.m` | drivers |

**Copied unchanged** from `IHW3_Code` (engine-agnostic): `AircraftState.m`,
`ConstraintSetImporter.m`, `MissionProfileReader.m`, `json_as_struct_array.m`,
`GeometryBase.m`, `AerodynamicsBase.m`, `WeightsBase.m`, `SizingSteps.m`,
`wing_fuel_check.m`, `landing_gear_loads.m`. Their comments still mention the
TTPA (for example, `SizingSteps` cites 0.45 as the TTPA's best relaxation);
the equations are unchanged and apply as they are.

## Adapting it to your aircraft

1. **Requirements** (`constraints.conditions`, `missions`): field lengths,
   climb gradients, cruise Mach and altitude, ceiling, range, hold, payload.
   Add or remove conditions freely; `type` picks the equation. A sustained
   turn is a `flight` condition with `n > 1`.
2. **Engine** (`propulsion`): engine count, bypass ratio, throttle ratio,
   installation loss, and the engine type:

   | Engine | `lapse_model` and `tsfc_model` | typical `bypass_ratio` | typical `throttle_ratio` |
   |---|---|---|---|
   | High-bypass turbofan | `"mattingly_hbtf"` | 4 and up | 1.0–1.1 |
   | Low-bypass turbofan, dry, subsonic | `"mattingly_lbtf"` | below about 1.5 | 1.0–1.08 |

   Set both keys to the same type and set `bypass_ratio` to match, because
   the engine weight and diameter regressions read it. The low-bypass
   branch has **no afterburner**. When you have a candidate engine, replace
   the Mattingly correlations with its data (add a branch in
   `TfProp.thrust_lapse` and `TfProp.tsfc`).
3. **Weights**: calibrate the unit weights against one or two existing
   airplanes of your class (see the JSON note). This is the least certain
   block, and the converged weight is sensitive to it.
4. **Geometry**: AR, sweep, t/c, tail arms, fuselage size off your layout.
   **Airfoil and sweep:** `aerodynamics.airfoil_type` is `"supercritical"`
   (Korn κ 0.95) or `"conventional"` (κ 0.87); `"custom"` uses
   `korn_technology_factor`. `geometry.sweep_quarter_chord_deg` is the sweep.
   With `aerodynamics.CLmax_reference_sweep_deg` set, all three CLmax values
   are scaled by cos Λ / cos Λ_ref, so changing the sweep costs CLmax as well
   as raising the drag-divergence Mach. Both can be overridden for one run
   without editing the file, and traded with `airfoil_sweep_study`:

   ```matlab
   run_tf_constraints('json', 'prm2_m090_requirements.json', 'airfoil', 'conventional', 'sweep_deg', 35);
   run_tf_sizing('json', 'prm2_m090_requirements.json', 'airfoil', 'supercritical', 'sweep_deg', 30);
   T = airfoil_sweep_study('json', 'prm2_m090_requirements.json', 'sweeps', 20:5:40);
   ```

   **Several concepts side by side** (different aerodynamics, so one tile each):

   ```matlab
   compare_concepts({'prm2_concept2_rfp_requirements.json', 'prm2_concept3_rfp_requirements.json', ...
                     'prm2_concept5_rfp_requirements.json', 'prm2_concept6_rfp_requirements.json'}, 'save_png', true);
   ```

   **Several engines on one diagram** (each file = same airplane, different engine):

   ```matlab
   compare_engine_points({'prm2_f124_requirements.json', 'prm2_f404_requirements.json'}, 'save_png', true);
   compare_engine_points({'prm2_f124_requirements.json', 'prm2_f404_requirements.json'}, ...
                         'mode', 'common_weight', 'W_TO', 14000);
   ```
   Not modelled: the wing-weight penalty of sweep and either airfoil's
   handling qualities.
   Set `geometry.nacelle.installation` to `"buried"` for engines inside the
   fuselage (no nacelle wetted area) or leave it at `"pod"`.
   **Wing weight that responds to the design:** set
   `weights.wing_weight_method` to `"raymer_fighter"` and add a
   `weights.wing_raymer` block. The block holds `limit_load_factor`,
   `ultimate_factor`, `control_surface_fraction`, `K_dw`, `K_vs` and
   `calibration_factor`. The wing then follows Raymer Eq. 15.1, which scales as
   √(W·N_z) · S^0.622 · AR^0.785 · (t/c)^−0.4 / cos Λ, instead of 9 psf × exposed
   area.
   - `limit_load_factor: "auto"` designs the structure to the largest positive
     load factor in the file's own requirements. That gives 4.0 g for the PRM2
     thresholds, 6.0 g for the objectives and 7.0 g with the +7 g instantaneous
     requirement, as RFP-3-P02-R03.1.1 ties the limit to the sustained g.
   - Set `calibration_factor` so the equation matches the unit method on a
     reference airplane. The PRM2 concept files use **1.1448, anchored at the
     T-45C** at +7.33 g (the commonly quoted NATOPS figure, unverified). With
     that wing, the PRM2 files' all-else fraction (0.2336) reproduces the L-39C
     empty weight exactly and the T-45C within +0.8 %.
5. **Single-engine airplanes**: set `n_engines` to 1 and remove every
   condition with `oei: true`; `TfProp` stops with a clear error if one is
   left (an engine-out requirement would otherwise give T/W = Inf). Replace
   the FAR 25 set with the requirements that do apply, for example a
   `landing` condition with `distance_is_actual: true` (military landing
   distance, no FAR 25 1/0.6 factor), a carrier `approach_speed` wall, a wave-off `climb_gradient` with
   `oei: false`, and `flight` conditions for speed, ceiling, Ps and turns.
6. **Engine calibration**: `propulsion.tsfc_scale` (default 1) multiplies the
   Mattingly TSFC, so it can start from a real engine's SLS figure; the
   Raymer weight, length and diameter coefficients can be rescaled the same
   way (see the T-45C file).
6a. **Real (fixed) engine**: set `propulsion.engine_sizing` to `"fixed"` and
   give `propulsion.fixed_engine` (`name`, `thrust_lbf` uninstalled,
   `dry_weight_lbf`, `length_ft`, `diameter_ft`). T_SL is then held at
   n × thrust × (1 − installation loss), the engine size comes from the data,
   and the sizing loop closes only W_TO and the wing, taking the smallest
   wing the engine's T/W allows (`fixed_engine_wing_loading`). The default,
   `"rubber"`, is the old behaviour.
6b. **Several missions**: put more than one profile in `missions` (or list
   them in `sizing.missions`). Every pass flies them all at the same W_TO and
   carries the fuel of the one that needs most; `run_tf_sizing` prints a
   segment table for each and marks the governing one.
6c. **Time-and-thrust segments**: `"type": "fixed_thrust"` with `alt_ft`,
   `time_min`, a speed (`mach`, `ktas`, `keas` or `speed_over_stall` in a
   `config`) and `thrust` = `"military"`, `"max_continuous"`, `"drag"`
   (× `drag_factor`, for patterns and approaches) or `"idle"` (needs
   `fuel_flow_pph`). A `loiter` with no speed is flown at maximum-endurance
   speed (L/D max), as the RFP reserves specify.
7. Check the post-sizing output: wing fuel volume (the placeholder airplane
   is about 24 % short and would need a fuselage tank), the drag-divergence
   Mach against your cruise Mach, and the engine-regression range.

### V-tail (Concept 5)

Set `geometry.tail_type` to `"v_tail"` and give a `geometry.v_tail` block in place of
`horizontal_tail` and `vertical_tail`. Optionally add `weights.v_tail`,
`aerodynamics.tail_interference_ratio` and `aerodynamics.trim_drag`.

- **Sizing.** The V is sized from the two volume coefficients by the
  Purser–Campbell projection rule, S_V cos²Γ ≥ S_h and S_V sin²Γ ≥ S_v.
- **Weight.** Raymer's fighter horizontal-tail equation is applied to the whole V,
  anchored at the T-45C.
- **Trim drag.** The tail's induced drag uses the V's projected span b_V cos Γ.

All equations, assumptions and the effect on Concept 5 are in
[V_TAIL_MODEL.md](V_TAIL_MODEL.md). The `trim_drag` block works for a
conventional tail too; all four PRM2 concept files enable it.

### Twin canted fins (Concept 2)

Set `geometry.tail_type` to `"twin_vertical"` and add `n_fins` and `cant_deg` to
`vertical_tail`. Its `volume_coefficient` is then the effective (yaw) value on the
fins' vertical projection.

- **Sizing.** The fins' true area is S_v,req / cos²(cant).
- **Weight.** Optional `weights.twin_vertical` block: Raymer's fighter vertical-tail
  equation applied to each fin, anchored at the T-45C.

See [TWIN_TAIL_MODEL.md](TWIN_TAIL_MODEL.md).

## Limitations worth knowing

- **Climb is quasi-steady**: the energy spent accelerating during the climb
  is neglected, so time and fuel to climb are slightly optimistic.
- **Only two thrust ratings** (takeoff, max continuous). Max climb and max
  cruise are modelled as one of those; keep the mission climb and the
  ceiling constraint on the same rating (they are, by default).
- **Generic engine models.** The Mattingly lapse and TSFC are textbook
  correlations; a real engine deck can differ by 10–20 % at altitude, which
  moves the cruise and ceiling curves directly.
- **Oswald e jumps at 30° leading-edge sweep**, where Raymer's two formulas
  meet imperfectly.
- **The FAR 25 field-length correlations are Roskam's statistical fits**:
  good for preliminary sizing, not for certification-level performance.
- **Starting guesses matter for the first pass only.** A wing guess far too
  big for the weight guess can make the very first closure infeasible; the
  driver then stops and says so. `S_guess ≈ W_guess / 62` works well.
