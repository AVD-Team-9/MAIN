classdef (Abstract) ThrustPropulsionBase < handle
%THRUSTPROPULSIONBASE  Abstract interface for THRUST-producing propulsion.
%
%   The turbofan counterpart of PropulsionBase2. PropulsionBase2 is written
%   around SHAFT POWER: its abstract property is P_SL [hp] and its methods
%   return a power lapse, a brake-specific fuel consumption [lbm/(hp*hr)]
%   and a propeller efficiency. None of that describes a jet engine, which
%   delivers THRUST directly, burns fuel per pound of thrust (TSFC), and has
%   no propeller. So a separate base class is cleaner than bending the
%   power one.
%
%   A concrete class (TfProp) must provide:
%
%   properties
%       engine_type   text describing the engine
%       T_SL          installed, sea-level-static, takeoff-rated thrust of
%                     ALL engines [lbf] - the sizing-loop state that
%                     replaces P_SL
%       n_engines     number of engines [-]
%
%   methods
%       alpha = thrust_lapse(obj, state, rating)
%           Thrust available at the flight state and rating, divided by
%           T_SL [-]. Multiply by T_SL to get lbf.
%       kT = thrust_ratio(obj, state, con)
%           thrust_lapse plus every other reason the thrust at a constraint
%           condition differs from T_SL: one engine out, throttle setting.
%       c = tsfc(obj, state)
%           Thrust-specific fuel consumption [1/hr] = lbm of fuel per hour
%           per lbf of thrust.

    properties (Abstract)
        engine_type
        T_SL
        n_engines
    end

    methods (Abstract)
        alpha = thrust_lapse(obj, state, rating)
        kT    = thrust_ratio(obj, state, con)
        c     = tsfc(obj, state)
    end
end
