function [TW_min, extra] = constraint_flight(WS, obj, con_no)
%CONSTRAINT_FLIGHT  Mattingly's master equation -> minimum (T/W)_TO.
%
%   [TW_min, extra] = constraint_flight(WS, obj, con_no)
%
%   One function for every IN-FLIGHT requirement: cruise speed, ceiling /
%   rate of climb at altitude, sustained turn, acceleration. The condition
%   names the altitude, Mach, load factor n and specific excess power Ps;
%   what kind of requirement it is follows from those numbers:
%
%       cruise            n = 1, Ps = 0            thrust = drag
%       ceiling / ROC     n = 1, Ps = climb rate   thrust = drag + W dh/dt / V
%       sustained turn    n > 1, Ps = 0
%
%   REPLACES constraint_cruise_speed, which was written for a propeller:
%   its power-index form is a piston correlation, and its drag-based form
%   divided by 550 eta_p V to turn drag into shaft power. A jet's thrust IS
%   the force, so the power balance becomes a force balance.
%
%   THE MASTER EQUATION [Mattingly, Heiser & Pratt, Aircraft Engine Design,
%   2nd ed., Eq. 2.11]:
%
%       (T/W)_TO = (beta/alpha) [ (q/(beta (W/S)_TO)) CD + Ps/V ]
%
%   with   q  = 0.5 rho V^2 at the condition
%          CL = n beta (W/S)_TO / q                    lift = n x weight
%          CD = CD0 + K1 CL^2 + K2 CL + CD_wave(M, CL) TfAero.CD, clean
%          alpha = kT from TfProp.thrust_ratio (lapse, rating, OEI, throttle)
%
%   The first term in the bracket is drag/weight, the second the excess
%   thrust needed to climb (or accelerate) at Ps. Multiplying by beta/alpha
%   refers both to takeoff weight and to sea-level-static thrust.
%
%   SHAPE ON THE DIAGRAM. q CD0 / (beta W/S) falls with W/S (parasite drag
%   is a smaller share of a heavily loaded wing) while K1 n^2 beta (W/S)/q
%   rises (induced drag), so the curve is a BUCKET. Its bottom is at the W/S
%   where the condition is flown at best L/D. The wave-drag term steepens
%   the right side at high Mach, because a heavier wing loading means a
%   higher CL, which lowers the drag-divergence Mach number.
%
%   CL ABOVE CLmax. Where the requested n and W/S need more lift than the
%   clean CLmax, the condition cannot be flown at all, and TW_min is set
%   to Inf there (it then sits off the top of the diagram).

    con   = get_con(con_no, obj.cons);
    state = get_state(con.alt, con.mach);

    V  = state.V;                               % ft/s, true airspeed
    q  = state.q;                               % lbf/ft^2
    kT = obj.prop.thrust_ratio(state, con);     % alpha: thrust lapse etc.

    CL = con.n .* con.beta .* WS ./ q;          % lift = n * weight
    CD = obj.aero.CD(state, CL);                % includes wave drag

    TW_min = (con.beta ./ kT) .* ( q .* CD ./ (con.beta .* WS) + con.Ps_fps ./ V );

    % Not flyable: the wing would have to exceed its clean CLmax
    CLmax = obj.aero.CLmax_clean;
    TW_min(CL > CLmax) = Inf;

    extra = struct('q', q, 'V_fps', V, 'kT', kT, 'CL', CL, 'CD', CD, ...
                   'LD', CL ./ CD, 'CD_wave', obj.aero.wave_drag(state.mach, CL));
end
