function [TW_env, WS_max, driving, WS_best, TW_best] = matching_envelope(WS, obj)
%MATCHING_ENVELOPE  Combine every constraint into the T/W - W/S diagram.
%
%   [TW_env, WS_max, driving, WS_best, TW_best] = matching_envelope(WS, obj)
%
%   Outputs
%     TW_env   1xM LARGEST of the thrust-loading requirements at each W/S [-]
%     WS_max   tightest wing-loading wall [lbf/ft^2], Inf if there is none
%     driving  1xM label of the condition that sets TW_env
%     WS_best  wing loading of the least-thrust feasible point [lbf/ft^2]
%     TW_best  its thrust loading [-]   (both NaN if nothing is feasible)
%
%   THE FLIP. For the propeller TTPA the feasible region was BELOW every
%   W/P curve, so the envelope was the MINIMUM of the curves and the best
%   point was the HIGHEST W/P (smallest engine). For a jet:
%
%       every condition demands T/W >= its curve
%       -> feasible region is ABOVE every curve
%       -> envelope   = MAXIMUM of the curves at each W/S
%       -> feasible   = W/S at or below the tightest wall (unchanged)
%       -> best point = the feasible W/S where the envelope is LOWEST
%                       (least thrust = smallest engine)
%
%   Three lines differ from the TTPA file: min -> max for the envelope,
%   -Inf -> +Inf for the infeasible points, and max -> min for the best point.
%
%   The best point sits exactly on the constraints with no margin. It is
%   the smallest engine the requirements allow - the starting point for
%   choosing a design point, not the design point itself.

    WS = reshape(WS, 1, []);

    [TW_limits, WS_walls, names] = run_constraints(WS, obj);

    % Conditions that limit the thrust loading
    is_thrust = ~all(isnan(TW_limits), 2);
    if ~any(is_thrust)
        error('matching_envelope:NoThrustConstraint', ...
            'The constraint set has no condition that limits the thrust loading.');
    end

    % Envelope = the most demanding requirement at each W/S
    [TW_env, idx] = max(TW_limits(is_thrust, :), [], 1);
    thrust_names  = names(is_thrust);
    driving       = thrust_names(idx);

    % Tightest wing-loading wall
    if all(isnan(WS_walls))
        WS_max = Inf;
    else
        WS_max = min(WS_walls(~isnan(WS_walls)));
    end

    % Least-thrust point left of the wall. Points beyond the wall are made
    % +Inf so min() never picks them.
    usable = TW_env;
    usable(WS > WS_max) = Inf;

    if all(isinf(usable))
        WS_best = NaN;
        TW_best = NaN;
    else
        [TW_best, j] = min(usable);
        WS_best      = WS(j);
    end
end
