function [TW_limits, WS_walls, names, types] = run_constraints(WS, obj)
%RUN_CONSTRAINTS  Every constraint condition, over the wing-loading sweep.
%
%   [TW_limits, WS_walls, names, types] = run_constraints(WS, obj)
%
%   Inputs
%     WS         wing-loading sweep (W/S)_TO [lbf/ft^2], 1xM
%     obj        discipline bundle from tf_disciplines
%   Outputs
%     TW_limits  NxM MINIMUM (T/W)_TO each condition requires [-]. A
%                condition that only limits the wing loading (landing) has a
%                row of NaN.
%     WS_walls   1xN MAXIMUM (W/S)_TO each condition allows [lbf/ft^2]. NaN
%                for conditions that limit the thrust loading.
%     names      1xN condition labels
%     types      1xN condition types
%
%   Same dispatcher as the TTPA run_constraints, with two differences:
%     1. The thrust-loading rows are LOWER bounds (minimum T/W), where the
%        TTPA's power-loading rows were UPPER bounds (maximum W/P). For a
%        jet a bigger number on the vertical axis means a BIGGER engine;
%        for the propeller W/P it meant a SMALLER one.
%     2. The condition types are the jet ones:
%          "takeoff"         constraint_takeoff_far25   FAR 25 field length
%          "landing"         constraint_landing_far25   FAR 25 field length (wall)
%          "climb_gradient"  constraint_climb_far25     FAR 25.111/.119/.121
%          "flight"          constraint_flight          master equation
%          "approach_speed"  constraint_approach_speed  max approach speed (wall)
%                                                       - carrier aircraft
%          "instantaneous_turn" constraint_instantaneous_turn  lift-limited
%                                                       +/- g (wall)
%
%   Nothing is cached: every call reads the conditions and the models again,
%   so a change in S_ref or T_SL shows up on the next call.

    WS    = reshape(WS, 1, []);
    n_con = numel(obj.cons);

    TW_limits = NaN(n_con, numel(WS));
    WS_walls  = NaN(1, n_con);
    names     = strings(1, n_con);
    types     = strings(1, n_con);

    for k = 1:n_con
        con      = get_con(k, obj.cons);
        names(k) = con.name;
        types(k) = con.type;

        switch con.type
            case "takeoff"
                TW_limits(k,:) = constraint_takeoff_far25(WS, obj, k);
            case "landing"
                WS_walls(k)    = constraint_landing_far25(obj, k);
            case "climb_gradient"
                TW_limits(k,:) = constraint_climb_far25(WS, obj, k);
            case "flight"
                TW_limits(k,:) = constraint_flight(WS, obj, k);
            case "approach_speed"
                WS_walls(k)    = constraint_approach_speed(obj, k);
            case "instantaneous_turn"
                WS_walls(k)    = constraint_instantaneous_turn(obj, k);
            otherwise
                error('run_constraints:UndefinedCondition', ...
                    'Constraint condition type "%s" is not defined.', con.type);
        end
    end
end
