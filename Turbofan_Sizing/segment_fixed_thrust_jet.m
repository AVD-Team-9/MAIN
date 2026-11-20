function [W_out, fuel_used, WF, info] = segment_fixed_thrust_jet(W_in, obj, seg_no)
%SEGMENT_FIXED_THRUST_JET  Fly for a fixed TIME at a stated thrust setting.
%
%   [W_out, fuel_used, WF, info] = segment_fixed_thrust_jet(W_in, obj, seg_no)
%
%   For mission legs that are defined by TIME and POWER SETTING rather than
%   by a distance or by a best-range/best-endurance condition - exactly how
%   the PRM2 RFP writes most of its training mission:
%       "engine start, warmup and taxi for 15 minutes at idle thrust"
%       "takeoff and acceleration ... for 1 minute at takeoff thrust"
%       "basic maneuvers at 10,000 ft for 30 minutes at military thrust"
%       "30 minutes of FCLP touch-and-go landings at approach speed"
%       "10 minutes of approach and landing"
%   None of these fits the Breguet segments (cruise, loiter), which assume
%   thrust = drag at a chosen speed, or the fixed weight fractions, which
%   do not know how big the engine is.
%
%   FUEL. Fuel flow = TSFC x thrust, integrated over the time in
%   n_sub = 10 steps (so a thrust tied to drag can follow the falling weight):
%       dW = - c(state) T dt
%
%   THRUST SETTINGS (JSON key "thrust", default "military"):
%     "military"        full dry rating: T = T_SL alpha(state, takeoff) x throttle
%     "max_continuous"  T = T_SL alpha(state, max_continuous) x throttle
%     "drag"            T = drag_factor x level-flight drag in the polar named
%                       by "config" (e.g. "landing_flaps_gear_down" for FCLP
%                       circuits); drag_factor > 1 covers the climbs and
%                       go-arounds of a touch-and-go pattern. Errors if that
%                       needs more than military thrust.
%     "idle"            needs "fuel_flow_pph": the Mattingly TSFC correlations
%                       are MAXIMUM-power fits and are not valid at idle.
%   "fuel_flow_pph" (total, all engines, lb/hr) overrides any setting - use it
%   whenever you have the engine's fuel flow from its data.
%
%   SPEED: "speed_over_stall" (k x stall speed in "config" at the start
%   weight), or "mach", "ktas" or "keas" (any one; default static, Mach 0,
%   for ground segments). ALTITUDE: alt_ft. TIME: time_min.
%
%   WHAT IT IGNORES. Speed and altitude changes inside the segment (a
%   maneuvering block is flown as one steady average condition), and the
%   rise of TSFC at part power for the "drag" setting (the correlation is a
%   full-power one, so part-power fuel is slightly OPTIMISTIC).

    seg = get_miss_seg(seg_no, obj.miss);
    if isempty(seg.time_min)
        error('segment_fixed_thrust_jet:NoTime', ...
            'fixed_thrust segment %d (%s) needs time_min.', seg_no, seg.name);
    end

    % ---- flight condition -------------------------------------------------
    % speed_over_stall: fly at k x the stall speed of seg.config at the
    % segment's START weight - an approach or touch-and-go speed that follows
    % the airplane the loop is sizing, instead of a fixed number of knots
    % that could fall below the stall speed of a small wing.
    s0 = get_state(seg.alt, 0);
    if ~isempty(seg.speed_over_stall)
        con0  = struct('config', seg.config, 'engine_out_drag', false);
        CLmx  = obj.aero.get_CLmax(s0, con0);
        Vs    = sqrt(2 * W_in / (s0.rho * obj.geom.S_ref * CLmx));
        M     = seg.speed_over_stall * Vs / s0.a;
    elseif ~isempty(seg.mach)
        M = seg.mach;
    elseif ~isempty(seg.ktas)
        M = seg.ktas * 1.68781 / s0.a;
    elseif ~isempty(seg.keas)
        M = seg.keas * 1.68781 / sqrt(s0.sigma) / s0.a;
    else
        M = 0;                                   % on the ground
    end
    st = get_state(seg.alt, M);
    V  = st.V;
    q  = st.q;
    S  = obj.geom.S_ref;

    rating = "takeoff";
    if seg.thrust == "max_continuous", rating = "max_continuous"; end
    T_mil = obj.prop.T_SL * obj.prop.thrust_lapse(st, "takeoff");   % military, all engines

    n_sub = 10;
    dt_hr = seg.time_min / 60 / n_sub;
    W = W_in;  CL_sum = 0;  LD_sum = 0;  T_sum = 0;
    for i = 1:n_sub
        CL = NaN;  LD = NaN;
        if ~isempty(seg.fuel_flow_pph)
            ff = seg.fuel_flow_pph;                       % lb/hr, from engine data
            T  = NaN;
        else
            switch seg.thrust
                case {"military", "max_continuous"}
                    T = obj.prop.T_SL * obj.prop.thrust_lapse(st, rating) * seg.throttle;
                case "drag"
                    if V <= 0
                        error('segment_fixed_thrust_jet:NoSpeed', ...
                            'Segment %d (%s): thrust = "drag" needs a speed.', seg_no, seg.name);
                    end
                    con = struct('config', seg.config, 'engine_out_drag', false);
                    cfg = obj.aero.get_config_polar(st, con);
                    CL  = W / (q * S);
                    if CL > cfg.CLmax
                        error('segment_fixed_thrust_jet:BelowStall', ...
                            ['Segment %d (%s): CL %.2f exceeds CLmax %.2f in %s - ', ...
                             'the speed is below the stall speed.'], seg_no, seg.name, CL, cfg.CLmax, seg.config);
                    end
                    CD  = obj.aero.CD(st, CL, cfg);
                    LD  = CL / CD;
                    T   = seg.drag_factor * q * S * CD;
                    if T > T_mil
                        error('segment_fixed_thrust_jet:NotEnoughThrust', ...
                            ['Segment %d (%s) needs %.0f lbf (%.2f x drag) but military ', ...
                             'thrust there is %.0f lbf.'], seg_no, seg.name, T, seg.drag_factor, T_mil);
                    end
                case "idle"
                    error('segment_fixed_thrust_jet:IdleNeedsFuelFlow', ...
                        ['Segment %d (%s): thrust = "idle" needs fuel_flow_pph (the TSFC ', ...
                         'correlations are not valid at idle).'], seg_no, seg.name);
                otherwise
                    error('segment_fixed_thrust_jet:UndefinedThrust', ...
                        'Segment %d: thrust "%s" is not defined.', seg_no, seg.thrust);
            end
            ff = obj.prop.tsfc(st) * T;                   % lb/hr
        end
        W = W - ff * dt_hr;
        CL_sum = CL_sum + CL;  LD_sum = LD_sum + LD;  T_sum = T_sum + T;
    end

    W_out     = W;
    fuel_used = W_in - W;
    WF        = W / W_in;
    info = struct('type', "fixed_thrust", 'CL', CL_sum / n_sub, 'LD', LD_sum / n_sub, ...
                  'V_fps', V, 'time_hr', seg.time_min / 60, 'n_sub', n_sub, ...
                  'dist_nm', V * seg.time_min * 60 / 6076.115);
end
