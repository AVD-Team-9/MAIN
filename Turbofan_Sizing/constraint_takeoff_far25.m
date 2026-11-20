function [TW_min, TOP25] = constraint_takeoff_far25(WS, obj, con_no)
%CONSTRAINT_TAKEOFF_FAR25  FAR 25 takeoff field length -> minimum (T/W)_TO.
%
%   [TW_min, TOP25] = constraint_takeoff_far25(WS, obj, con_no)
%
%   Inputs
%     WS      takeoff wing loading (W/S)_TO [lbf/ft^2], scalar or row vector
%     obj     discipline bundle from tf_disciplines
%     con_no  number of the takeoff condition in obj.cons
%   Outputs
%     TW_min  smallest (T/W)_TO that meets the field length [-]
%             (installed sea-level-static takeoff thrust / takeoff weight)
%     TOP25   FAR 25 takeoff parameter the field length requires [lbf/ft^2]
%
%   REPLACES constraint_takeoff (FAR 23, Roskam Sec. 3.1). The TTPA used the
%   FAR 23 ground-roll correlation S_TGR = 4.9 TOP23 + 0.009 TOP23^2, with
%   TOP23 = (W/S)(W/P)/(sigma CLmax) - a POWER-loading parameter. A
%   transport-category jet is certificated to a FAR 25 TAKEOFF FIELD LENGTH,
%   which already includes the engine failure at V1 and the climb to 35 ft
%   (the balanced field length). Roskam Part I, Sec. 3.4.1 correlates it as
%
%       s_TOFL = 37.5 TOP25                       [ft]
%       TOP25  = (W/S)_TO / ( sigma CLmax_TO (T/W)_TO )      [lbf/ft^2]
%
%   so the requirement s_TOFL <= distance gives a LOWER bound on T/W:
%
%       (T/W)_TO >= 37.5 (W/S)_TO / ( sigma CLmax_TO s_TOFL )
%
%   This is a straight line through the origin on the T/W - W/S diagram:
%   a heavier wing loading needs proportionally more thrust to get off in
%   the same distance.
%
%   ALTITUDE. The correlation is written in terms of the thrust actually
%   available on the runway. At a sea-level airport that is T_SL. At a high
%   or hot airport it is less, so the requirement is divided by the static
%   thrust lapse at the airport altitude (Mach 0). At sea level that factor
%   is exactly 1 and this reduces to Roskam's equation.

    con   = get_con(con_no, obj.cons);
    state = get_state(con.alt, 0);                 % static: on the runway

    CLmax_TO = obj.aero.get_CLmax(state, con);     % takeoff-flap CLmax

    % Required takeoff parameter from the field length
    TOP25 = con.distance_ft / 37.5;                % lbf/ft^2

    % Thrust available on THIS runway relative to T_SL (1 at sea level)
    kT = obj.prop.thrust_ratio(state, con);

    % T/W at the runway, then referred to sea-level-static thrust.
    % beta is 1 for takeoff by definition, but it is kept so the form matches
    % the other constraints: thrust loading at the condition = (T/W)_TO kT / beta.
    TW_runway = WS ./ (state.sigma * CLmax_TO * TOP25);
    TW_min    = TW_runway * con.beta / kT;
end
