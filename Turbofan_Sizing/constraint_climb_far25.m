function [TW_min, LD, CL, cfg] = constraint_climb_far25(WS, obj, con_no)
%CONSTRAINT_CLIMB_FAR25  FAR 25 climb-gradient requirement -> minimum (T/W)_TO.
%
%   [TW_min, LD, CL, cfg] = constraint_climb_far25(WS, obj, con_no)
%
%   Inputs
%     WS      takeoff wing loading [lbf/ft^2] (only sets the output size: the
%             climb-gradient requirement does not depend on W/S)
%     obj     discipline bundle
%     con_no  number of the climb condition in obj.cons
%   Outputs
%     TW_min  smallest (T/W)_TO that meets the gradient [-], same size as WS
%     LD      lift-to-drag ratio in the climb
%     CL      lift coefficient the climb is flown at
%     cfg     configuration polar
%
%   REPLACES constraint_climb. The TTPA wrote the climb through Roskam's
%   climb-gradient PARAMETER, CGRP = 18.97 eta_p sqrt(sigma)/((W/P) sqrt(W/S)),
%   which exists because a propeller's thrust falls with speed, so the
%   power needed depends on W/S. A jet's thrust is nearly independent of
%   speed at low Mach, so the steady-climb force balance can be written
%   directly (small angles, sin(gamma) ~ G):
%
%       T - D = W sin(gamma)   ->   T/W = G + D/L = G + 1/(L/D)
%
%   [Roskam Part I, Sec. 3.4.3; Raymer Sec. 5.3.] Referring the thrust to
%   T_SL and the weight to W_TO, with beta = W/W_TO and kT the thrust ratio
%   from TfProp (lapse x OEI factor x throttle):
%
%       (T/W)_TO = (beta / kT) (G + 1/(L/D))
%
%   For an OEI condition kT carries (N-1)/N, so dividing by it gives
%   Roskam's N/(N-1) factor: a twin needs TWICE the thrust of the climb
%   force balance, which is why the second segment so often sizes a twin.
%
%   THE CLIMB SPEED. FAR 25 fixes each climb at a multiple k of the stall
%   speed (V2 = 1.2 Vs for the takeoff segments, 1.25 en route, 1.3 balked
%   landing, 1.5 approach). Lift = weight at both speeds gives
%
%       CL = CLmax / k^2
%
%   which replaces the TTPA's fixed CLmax - 0.2. L/D comes from the
%   configuration polar (flaps, gear and windmilling-engine increments).
%
%   Because nothing here depends on W/S, the curve is a HORIZONTAL LINE on
%   the diagram. The flight Mach number (con.mach) only enters through the
%   thrust lapse.

    con   = get_con(con_no, obj.cons);
    state = get_state(con.alt, con.mach);

    cfg = obj.aero.get_config_polar(state, con);

    k  = con.speed_over_stall;
    CL = cfg.CLmax / k^2;                         % FAR 25 climb speed
    CD = obj.aero.CD(state, CL, cfg);             % configuration polar
    LD = CL / CD;

    kT = obj.prop.thrust_ratio(state, con);       % lapse x (N-1)/N x throttle

    TW = (con.beta / kT) * (con.G + 1/LD);
    TW_min = TW * ones(size(WS));
end
