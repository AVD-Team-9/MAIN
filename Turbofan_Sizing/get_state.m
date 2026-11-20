function [state] = get_state(alt_ft, mach)
%GET_STATE  Standard-atmosphere flight state at an altitude AND a Mach number.
%
%   state = get_state(alt_ft)          Mach 0 (static)
%   state = get_state(alt_ft, mach)
%
%   Returns a struct with
%       state.alt      altitude [ft]
%       state.mach     flight Mach number [-]
%       state.rho      density [slug/ft^3]
%       state.sigma    density ratio rho/rho_SL [-]
%       state.T        static temperature [deg R]
%       state.P        static pressure [lbf/ft^2]
%       state.a        speed of sound [ft/s]
%       state.V        true airspeed = mach * a [ft/s]
%       state.q        dynamic pressure = 0.5 rho V^2 [lbf/ft^2]
%       state.theta    T / T_SL [-]
%       state.delta    P / P_SL [-]
%       state.theta_0  total-temperature ratio = theta (1 + 0.2 M^2)       [-]
%       state.delta_0  total-pressure ratio    = delta (1 + 0.2 M^2)^3.5   [-]
%                      [Mattingly, Aircraft Engine Design 2nd ed., Eq. 2.52]
%
%   WHAT CHANGED FROM THE TTPA VERSION, AND WHY
%   The piston TTPA only needed rho and sigma, because its power lapse
%   (1.132 sigma - 0.132) depends on density alone, so the TTPA get_state
%   took only an altitude and threw away everything else AircraftState
%   computes. A turbofan's thrust lapse depends on the TOTAL (stagnation)
%   temperature and pressure the engine face sees, theta_0 and delta_0,
%   which depend on the Mach number as well as the altitude. So:
%
%     1. get_state takes a Mach number (default 0, so old one-argument calls
%        still work and still return the static state).
%     2. It keeps theta_0 and delta_0 (and T, P, a, V, q) from AircraftState.
%     3. The cache is keyed on the PAIR (altitude, Mach), not on altitude
%        alone - otherwise a call at M 0.2 would return a cached M 0.75 state.
%
%   MEMOIZED, exactly like the TTPA version: the standard atmosphere is a
%   pure function of altitude and Mach, so the same inputs always give the
%   same answer. The cache is never invalidated; clear('get_state') drops it.

    arguments
        alt_ft (1,1) double {mustBeNonnegative}
        mach   (1,1) double {mustBeNonnegative} = 0
    end

    persistent cache rho_SL

    if isempty(cache)
        % containers.Map with a CHAR key. The key below joins altitude and
        % Mach at full double precision ('%.17g'), so two calls get the same
        % entry only when both inputs are bit-for-bit identical.
        cache  = containers.Map('KeyType', 'char', 'ValueType', 'any');
        % Sea-level density read from AircraftState rather than typed in,
        % so sigma comes out exactly 1.0 at sea level.
        rho_SL = AircraftState(0, 0).rho;
    end

    key = sprintf('%.17g|%.17g', alt_ft, mach);
    if isKey(cache, key)
        state = cache(key);
        return;
    end

    % AircraftState does the ISA (MATLAB atmosisa) and the unit conversion
    atm = AircraftState(alt_ft, mach);

    state.alt     = alt_ft;
    state.mach    = mach;
    state.rho     = atm.rho;
    state.sigma   = atm.rho / rho_SL;
    state.T       = atm.T_atm;
    state.P       = atm.P_atm;
    state.a       = atm.a;
    state.V       = atm.V;
    state.q       = atm.q;
    state.theta   = atm.theta;
    state.delta   = atm.delta;
    state.theta_0 = atm.theta_0;
    state.delta_0 = atm.delta_0;

    cache(key) = state;
end
