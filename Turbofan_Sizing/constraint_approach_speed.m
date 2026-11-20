function [WS_max, Vs_kts] = constraint_approach_speed(obj, con_no)
%CONSTRAINT_APPROACH_SPEED  Maximum approach speed -> maximum (W/S)_TO.
%
%   [WS_max, Vs_kts] = constraint_approach_speed(obj, con_no)
%
%   Outputs
%     WS_max   largest takeoff wing loading whose approach speed is at or
%              below the requirement [lbf/ft^2]
%     Vs_kts   the matching power-approach stall speed [KEAS]
%
%   WHY A SEPARATE CONSTRAINT. constraint_landing_far25 starts from a
%   FAR 25 landing FIELD LENGTH and turns it into an approach speed with
%   Roskam's s_FL = 0.3 V_A^2 correlation. A carrier airplane is not sized by
%   a runway: an arrested landing stops in a fixed distance whatever the
%   speed, and what limits the wing is the APPROACH SPEED itself, which sets
%   the energy the arresting gear must absorb and how well the pilot can fly
%   the glide slope to the wire. The requirement is therefore written
%   directly as a speed, and this function skips the field-length step.
%
%   THE EQUATIONS. The approach is flown at a fixed multiple k of the
%   power-approach stall speed (k = con.speed_over_stall):
%
%       V_s  = V_approach / k
%       (W/S)_L  = 0.5 rho_SL V_s^2 CLmax_L         V_s in FT/S, EQUIVALENT
%       (W/S)_TO = (W/S)_L / beta                    beta = W_landing / W_TO
%
%   Because the speed is EQUIVALENT airspeed (KEAS), the sea-level density
%   is used and the airfield altitude does not enter; a carrier deck is at
%   sea level anyway. Raymer (Sec. 5.3) gives k = 1.15 for Navy carrier
%   approaches, against 1.3 for FAR 25 civil approaches - check the value
%   in your own requirement document. Like the field-length landing, the
%   result does not depend on thrust, so it is a VERTICAL WALL on the
%   diagram.
%
%   JSON keys: type "approach_speed", approach_speed_kts, speed_over_stall,
%   config (normally "landing_flaps_gear_down"), beta.

    con = get_con(con_no, obj.cons);
    if isempty(con.approach_speed_kts)
        error('constraint_approach_speed:MissingSpeed', ...
            'Condition "%s" needs approach_speed_kts.', con.name);
    end

    state   = get_state(0, 0);                       % EAS -> sea-level density
    CLmax_L = obj.aero.get_CLmax(state, con);        % landing-configuration CLmax

    Vs_kts = con.approach_speed_kts / con.speed_over_stall;   % KEAS
    Vs_fps = Vs_kts * 1.68781;                                % ft/s

    WS_max_L = 0.5 * state.rho * Vs_fps^2 * CLmax_L;          % at landing weight
    WS_max   = WS_max_L / con.beta;                           % referred to W_TO
end
