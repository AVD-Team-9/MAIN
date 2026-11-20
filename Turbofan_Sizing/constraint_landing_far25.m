function [WS_max, Vs_kts, V_A_kts] = constraint_landing_far25(obj, con_no)
%CONSTRAINT_LANDING_FAR25  FAR 25 landing field length -> maximum (W/S)_TO.
%
%   [WS_max, Vs_kts, V_A_kts] = constraint_landing_far25(obj, con_no)
%
%   Outputs
%     WS_max   largest takeoff wing loading that meets the field length [lbf/ft^2]
%     Vs_kts   largest permitted landing-configuration stall speed [kt]
%     V_A_kts  the matching approach speed, 1.3 Vs [kt]
%
%   REPLACES constraint_landing (FAR 23). The FAR 23 version used the ground
%   roll, S_LGR = 0.265 Vs^2. FAR 25 certificates the whole landing FIELD
%   length (from 50 ft, divided by 0.6 for the operational factor), which
%   Roskam Part I, Sec. 3.4.2 correlates with the APPROACH speed:
%
%       s_FL = 0.3 V_A^2       [ft, with V_A in KNOTS]
%       V_A  = 1.3 V_s,L       (FAR 25 approach speed)
%
%   The stall speed then fixes the wing loading at landing weight,
%
%       (W/S)_L = 0.5 rho V_s^2 CLmax_L          (V_s in FT/S here)
%
%   and dividing by beta = W_L/W_TO refers it to takeoff weight. The landing
%   distance does not depend on thrust, so this is a VERTICAL WALL on the
%   diagram, exactly as in the TTPA code. Mind the units: the correlation is
%   in knots, the lift equation in ft/s.

    con   = get_con(con_no, obj.cons);
    state = get_state(con.alt, 0);

    CLmax_L = obj.aero.get_CLmax(state, con);

    % ACTUAL DISTANCE OPTION. Roskam's 0.3 V_A^2 is a FAR 25 FIELD length:
    % the demonstrated distance from 50 ft to a stop, divided by 0.6. A
    % military requirement (e.g. MIL-STD-3013B "landing distance") is the
    % actual distance with no such factor. With distance_is_actual = true
    % the requirement is converted to the field length it corresponds to,
    % s_FL = s_actual / 0.6, so the same correlation can be used. Without
    % this the wall would sit at a 40 % lower approach-speed-squared, far
    % too strict.
    s_FL = con.distance_ft;
    if con.distance_is_actual
        s_FL = con.distance_ft / 0.6;
    end
    V_A_kts = sqrt(s_FL / 0.3);                 % kt  (s_FL = 0.3 V_A^2)
    Vs_kts  = V_A_kts / 1.3;                    % kt
    Vs_fps  = Vs_kts * 1.68781;                 % ft/s  (1 kt = 1.68781 ft/s)

    WS_max_L = 0.5 * state.rho * Vs_fps^2 * CLmax_L;   % at landing weight
    WS_max   = WS_max_L / con.beta;                    % referred to W_TO
end
