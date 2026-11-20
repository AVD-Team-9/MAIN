Concept 6 constraint diagrams (F124-GA-100, both RFP missions, current inputs and code).
FIXED wing  = Concept_Analysis/concept_sizing: VSP wing area held, W_TO closed.
RUBBER wing = Turbofan_Sizing/run_tf_sizing: wing area chosen by the loop, W_TO closed.
Diamond = the design point; open circle = least-thrust corner of the feasible region.

RFP thresholds, FIXED wing (inputs/concept6_rfp.json): W_TO 14192 lb, S 215.5 ft^2 (fixed), W/S 65.9, T/W 0.428 -> all met

RFP thresholds, RUBBER wing (Turbofan_Sizing/prm2_concept6_rfp_requirements.json): converged W_TO 13785 lb, S 179.2 ft^2, W/S 76.9, T/W 0.441

Team requirements, FIXED wing (inputs/concept6_team.json): W_TO 14809 lb, S 215.5 ft^2 (fixed), W/S 68.7, T/W 0.411 -> FAILS Max speed M0.90 25k ft (R01.2)

Team requirements, RUBBER wing (Turbofan_Sizing/prm2_concept6_requirements.json): DID NOT CONVERGE (The first pass failed at the starting guesses (W_0 = 15000 lbf, T_SL = 6080 lbf, S_ref = 215.5 ft^2): At W0 = 15000 lbf the fixed engine (F124-GA-100)). Diagram drawn at the fixed-wing W_TO 14809 lb with the wing at the least-thrust W/S (156.3 ft^2); the diamond is the F124 there.

