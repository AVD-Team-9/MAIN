function row = sizing_pass(obj, W0, T0, S0, WS_sweep)
%SIZING_PASS  One pass of the turbofan sizing loop.
%
%   row = sizing_pass(obj, W0, T0, S0, WS_sweep)
%
%   Inputs
%     obj       discipline bundle. MUTATED: this pass writes T_SL and S_ref.
%     W0        takeoff weight this pass starts from [lbf]
%     T0        installed sea-level-static thrust, all engines [lbf]
%     S0        wing reference area [ft^2]
%     WS_sweep  wing-loading sweep for the constraint analysis [lbf/ft^2]
%   Output
%     row       record of this pass (one airplane, start to finish)
%
%   The same six steps as the TTPA sizing_pass. The ONE line that changes in
%   the logic is STEP 2's engine sizing:
%
%       propeller:  P_new = W0 / (W/P)     power loading is lb PER hp, so
%                                          divide - a bigger W/P is a smaller engine
%       turbofan:   T_new = W0 * (T/W)     thrust loading is thrust PER lb, so
%                                          multiply - a bigger T/W is a bigger engine
%
%     0  Write the airplane: T_SL, then S_ref. Every geometric, aerodynamic
%        and engine-size quantity is Dependent on these two.
%     1  Constraint analysis on THAT airplane -> least-thrust corner (W/S, T/W).
%        The cruise and ceiling curves read CD0 (from S_wet/S_ref) and the
%        wave drag, so the corner belongs to this wing.
%     2  Size the wing and the engine from the CURRENT weight:
%            S_new = W0 / (W/S)          T_new = W0 * (T/W)
%     3  Mission fuel at W0, flown by the airplane of step 0.
%     4  Empty weight at W0, built up from that airplane.
%     5  Close the takeoff weight:  W_new = W_payload / (1 - W_f/W0 - OEW/W0)
%     6  Residuals (before relaxation) and the record.

    % ---- STEP 0: write the states into the airplane -------------------------
    obj.prop.T_SL  = T0;
    obj.geom.S_ref = S0;

    % ---- STEP 1: constraint analysis on the current airplane ------------------
    [TW_env, WS_wall, driving, WS, TW] = matching_envelope(WS_sweep, obj);
    if isnan(WS)
        error('sizing_pass:NoFeasiblePoint', ...
            ['No wing loading on [%.1f, %.1f] lbf/ft^2 is feasible: the whole ', ...
             'sweep lies beyond the wing-loading wall at %.2f.'], ...
            WS_sweep(1), WS_sweep(end), WS_wall);
    end
    % Name of the condition that sets the envelope at the corner
    [~, j] = min(abs(WS_sweep - WS));
    drv = driving(j);

    % ---- STEP 2: size the wing and the engine from W0 -------------------------
    if obj.prop.is_fixed()
        % FIXED (real) ENGINE: thrust is not a design variable. T_SL stays at
        % the engine's installed thrust and only the WING is sized: the
        % smallest wing (largest W/S) at which the envelope stays below the
        % engine's T/W at this weight (fixed_engine_wing_loading).
        T0 = obj.prop.T_SL_fixed();
        obj.prop.T_SL = T0;
        [WS, TW, drv, ok] = fixed_engine_wing_loading(WS_sweep, TW_env, WS_wall, T0, W0, obj);
        if ~ok
            error('sizing_pass:FixedEngineTooSmall', ...
                'At W0 = %.0f lbf the fixed engine (%s) cannot meet the requirements: %s.', ...
                W0, obj.prop.fixed_engine.name, drv);
        end
        S_new = W0 / WS;
        T_new = T0;             % fixed: the residual on T is zero
    else
        S_new = W0 / WS;
        T_new = W0 * TW;        % MULTIPLY: T/W is thrust per pound of weight
    end

    % ---- STEP 3: mission fuel --------------------------------------------------
    [W_fuel, fuel_fraction] = mission_fuel(W0, obj);
    obj.wts.W_TO     = W0;
    obj.wts.W_energy = W_fuel;

    % ---- STEP 4: empty weight ----------------------------------------------------
    bd = obj.wts.OEW_breakdown(W0);

    % ---- STEP 5: takeoff-weight closure -----------------------------------------
    [W_new, denom] = SizingSteps.togw_update(obj.wts.payload(), bd.total, W_fuel, W0);
    if isnan(W_new)
        error('sizing_pass:closureInfeasible', ...
            ['Takeoff-weight closure is infeasible at W0 = %.1f lbf: denom = ', ...
             '1 - W_f/W0 - OEW/W0 = %.4f must be > 0.'], W0, denom);
    end

    % ---- STEP 6: the record --------------------------------------------------------
    g = obj.geom;  a = obj.aero;  p = obj.prop;

    row.iter          = NaN;           % set by sizing_mainloop
    row.W_TO          = W0;
    row.T_SL          = T0;
    row.S_ref         = S0;
    row.WS            = WS;
    row.TW            = TW;
    row.WS_wall       = WS_wall;
    row.driving       = string(drv);
    row.TW_env_min    = min(TW_env);

    row.b             = g.b;
    row.MAC           = g.MAC;
    row.S_ht          = g.S_ht;
    row.S_vt          = g.S_vt;
    row.S_wet         = g.S_wet;
    row.S_wet_over_S_ref = g.S_wet_over_S_ref;
    row.d_nacelle     = g.d_nacelle;
    row.l_nacelle     = g.l_nacelle;

    row.CD0           = a.CD0;
    row.e             = a.e;
    row.LD_max        = a.LD_max;

    row.T_engine      = p.T_engine;
    [row.W_eng_bare_total, row.W_eng_bare_each] = p.engine_weight();

    row.W_fuel        = W_fuel;
    row.fuel_fraction = fuel_fraction;
    row.W_payload     = obj.wts.payload();
    row.OEW_breakdown = bd;
    row.OEW           = bd.total;
    row.OEW_fraction  = bd.total / W0;
    row.OEW_statistical = obj.wts.OEW_statistical(W0);

    row.denom         = denom;
    row.W_TO_new      = W_new;
    row.T_SL_new      = T_new;
    row.S_ref_new     = S_new;
    row.res_W         = abs(W_new - W0) / W_new;
    row.res_T         = abs(T_new - T0) / T_new;
    row.res_S         = abs(S_new - S0) / S_new;
end
