function [WS_max, CL_limit, q] = constraint_instantaneous_turn(obj, con_no)
%CONSTRAINT_INSTANTANEOUS_TURN  Instantaneous (lift-limited) load factor -> maximum (W/S)_TO.
%
%   [WS_max, CL_limit, q] = constraint_instantaneous_turn(obj, con_no)
%
%   Outputs
%     WS_max    largest takeoff wing loading that can still reach the load
%               factor n at the condition's altitude and Mach [lbf/ft^2]
%     CL_limit  the lift coefficient that limits it: CLmax_clean for n > 0,
%               |CLmin_clean| for n < 0
%     q         dynamic pressure at the condition [lbf/ft^2]
%
%   WHY THIS IS A WING-LOADING WALL, NOT A THRUST CURVE. A SUSTAINED turn
%   (constraint_flight with n > 1, Ps = 0) must hold its speed, so thrust has
%   to equal the turn drag - that sizes the engine. An INSTANTANEOUS pull
%   (held for seconds, e.g. 7-15 s) is allowed to bleed speed and altitude:
%   the airplane trades energy for g, and what limits it is how much LIFT the
%   wing can make at that speed,
%
%       n W = q S CL_limit       ->     (W/S)_condition <= q CL_limit / |n|
%       (W/S)_TO = (W/S)_condition / beta
%
%   so it is a VERTICAL LINE on the diagram, like the landing wall, and it
%   does not depend on T_SL at all. [Raymer Sec. 17.4, instantaneous turn;
%   Mattingly AED Sec. 2.2.]
%
%   NEGATIVE g. For n < 0 the wing must make NEGATIVE lift, limited by the
%   most negative lift coefficient it can reach, CLmin_clean (JSON
%   aerodynamics.CLmin_clean, a negative number). A cambered section -
%   especially an aft-loaded SUPERCRITICAL one - reaches a much smaller
%   |CLmin| than CLmax, so -3 g can limit the wing before +7 g does.
%
%   WHAT IS NOT MODELLED
%     * CLmax falling with Mach (compressibility / buffet). The clean CLmax
%       (sweep-corrected) is used at the condition's Mach, so the wall is
%       optimistic above about M 0.6-0.7.
%     * The STRUCTURE. +7/-3 g is also a structural limit load (ultimate
%       = 1.5 x limit, RFP-4-P04-R02.1); the unit-weight build-up has no
%       load-factor term, so it does not get heavier with n.
%     * The time limit (7-15 s) - it sets how much speed is lost, which
%       this lift-limit view does not need, and the pilot's g tolerance.
%
%   JSON keys: type "instantaneous_turn", altitude_ft, mach, n (e.g. 7 or
%   -3), beta, config (default "clean").

    con   = get_con(con_no, obj.cons);
    state = get_state(con.alt, con.mach);
    q     = state.q;

    if con.n > 0
        CL_limit = obj.aero.get_CLmax(state, con);
    elseif con.n < 0
        if isnan(obj.aero.CLmin_clean)
            error('constraint_instantaneous_turn:NoCLmin', ...
                ['Condition "%s" asks for n = %.1f, which needs aerodynamics.CLmin_clean ', ...
                 '(the most negative lift coefficient, e.g. -0.8).'], con.name, con.n);
        end
        CL_limit = abs(obj.aero.CLmin_clean);
    else
        error('constraint_instantaneous_turn:ZeroLoadFactor', ...
            'Condition "%s" needs a nonzero load factor n.', con.name);
    end

    WS_max = q * CL_limit / (abs(con.n) * con.beta);
end
