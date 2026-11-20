function [W_out, fuel_used, WF, info] = segment_climb_jet(W_in, obj, seg_no)
%SEGMENT_CLIMB_JET  Jet climb flown on excess thrust, fuel burned at climb thrust.
%
%   [W_out, fuel_used, WF, info] = segment_climb_jet(W_in, obj, seg_no)
%
%   REPLACES segment_climb_L2. The propeller version computed rate of climb
%   from excess SHAFT POWER, (550 eta_p P - D V)/W, at the minimum-power-
%   required lift coefficient. A jet climbs on excess THRUST:
%
%       ROC = (T - D) V / W                     [ft/s]
%
%   [Raymer Sec. 17.3, Eq. 17.38 (jet rate of climb, steady); Anderson,
%   Aircraft Performance and Design, Sec. 5.3.] The fuel is burned at the
%   CLIMB THRUST, not at a shaft power:
%
%       dW = - c T dt / 3600                    c = TSFC [1/hr], T [lbf], dt [s]
%
%   THE SPEED SCHEDULE. A jet does not climb at a single lift coefficient.
%   It climbs at a constant EQUIVALENT airspeed (seg.keas, e.g. 250 KEAS)
%   until the true airspeed reaches a Mach limit (seg.mach, e.g. 0.70), then
%   holds that Mach number to the top. At each altitude band:
%
%       V_EAS-schedule = keas * 1.68781 / sqrt(sigma)   true airspeed, ft/s
%       V_Mach-limit   = mach * a
%       V              = the smaller of the two
%
%   Then, at the band's mid-height h and that V:
%       CL = W / (q S)               lift = weight
%       CD = TfAero.CD(state, CL)    including wave drag at the band's Mach
%       D  = q S CD
%       T  = T_SL * thrust_lapse(state, rating)
%       ROC, dt = dh/ROC, fuel = c T dt/3600
%
%   THE THRUST RATING. Jets climb at a MAXIMUM CLIMB rating, between
%   takeoff and max continuous. This model has only those two ratings, so
%   the segment's max_continuous key picks one (default false = the full
%   rating, which stands in for max climb). Keep it CONSISTENT with the
%   ceiling constraint in the constraint set: if the ceiling requirement is
%   evaluated at a higher rating than the mission climb, the engine the
%   constraint analysis sizes cannot actually fly the mission to altitude,
%   and this segment stops with 'CannotClimb'.
%
%   QUASI-STEADY: the energy that goes into ACCELERATING (true airspeed rises
%   as the air thins at constant EAS) is neglected, as in the steady-climb
%   equations. That makes the climb slightly optimistic; an energy-height
%   formulation (dh_e = dh + V dV/g) would fix it.
%
%   The climb starts where the PREVIOUS segment ended (sea level for the
%   first segment). A "climb" that does not gain height burns nothing.

    seg   = get_miss_seg(seg_no, obj.miss);
    h_end = seg.alt;

    % ---- where the climb starts --------------------------------------------
    if seg_no > 1
        prev    = get_miss_seg(seg_no - 1, obj.miss);
        h_start = prev.alt;
    else
        h_start = 0;
    end
    if h_end <= h_start
        WF = 1;  fuel_used = 0;  W_out = W_in;
        info = struct('type', "climb", 'CL', NaN, 'LD', NaN, 'V_fps', NaN, ...
                      'time_hr', 0, 'n_sub', 0, 'dist_nm', 0);
        return;
    end

    if isempty(seg.keas)
        error('segment_climb_jet:NoSpeed', 'Climb segment %d has no keas.', seg_no);
    end
    mach_cap = seg.mach;
    if isempty(mach_cap), mach_cap = Inf; end

    if seg.max_continuous
        rating = "max_continuous";
    else
        rating = "takeoff";            % stands in for maximum climb thrust
    end

    % ---- the bands and the airplane ------------------------------------------
    n_sub = obj.miss.climb_segments;
    dh    = (h_end - h_start) / n_sub;          % ft per band
    S     = obj.geom.S_ref;                      % THIS pass's wing

    W = W_in;  t_tot = 0;  x_tot = 0;  CL_sum = 0;  LD_sum = 0;  V_sum = 0;

    for i = 1:n_sub
        h    = h_start + (i - 0.5) * dh;         % band mid-height
        s0   = get_state(h, 0);                  % atmosphere only (rho, a)
        V    = min(seg.keas * 1.68781 / sqrt(s0.sigma), mach_cap * s0.a);
        st   = get_state(h, V / s0.a);           % full state at this Mach

        q    = st.q;
        CL   = W / (q * S);
        CD   = obj.aero.CD(st, CL);
        D    = q * S * CD;
        T    = obj.prop.T_SL * obj.prop.thrust_lapse(st, rating);
        ROC  = (T - D) * V / W;                  % ft/s

        if ~(isfinite(ROC) && ROC > 0)
            error('segment_climb_jet:CannotClimb', ...
                ['The airplane cannot climb at %.0f ft: T = %.0f lbf <= D = %.0f lbf. ', ...
                 'The engine is too small for this weight.'], h, T, D);
        end

        dt    = dh / ROC;                        % s
        c     = obj.prop.tsfc(st);               % 1/hr
        W     = W - c * T * dt / 3600;           % lbf
        t_tot = t_tot + dt;
        x_tot = x_tot + V * dt;                  % horizontal distance, ft (approx.)
        CL_sum = CL_sum + CL;  LD_sum = LD_sum + CL/CD;  V_sum = V_sum + V;
    end

    WF        = W / W_in;
    fuel_used = W_in - W;
    W_out     = W;
    info = struct('type', "climb", 'CL', CL_sum/n_sub, 'LD', LD_sum/n_sub, ...
                  'V_fps', V_sum/n_sub, 'time_hr', t_tot/3600, 'n_sub', n_sub, ...
                  'dist_nm', x_tot/6076.115);
end
