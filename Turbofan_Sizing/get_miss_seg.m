function [seg] = get_miss_seg(seg_no, miss)
%GET_MISS_SEG  One mission segment, with every optional key filled in.
%
%   seg = get_miss_seg(seg_no, miss) returns
%       seg.type             takeoff | climb | cruise | descent | loiter |
%                            landing | fixed_thrust
%       seg.name             label
%       seg.alt              END altitude of the segment [ft]
%       seg.mach             cruise Mach number                  (cruise)
%       seg.ktas             true airspeed [kt]                  (loiter)
%       seg.keas             equivalent airspeed [kt]            (climb)
%       seg.time_min         duration [min]                      (loiter)
%       seg.dist             distance [nmi]                      (cruise)
%       seg.weight_fraction  W_end / W_start                     (fixed-fraction segments)
%       seg.max_continuous   true = climb at max-continuous thrust,
%                            false (default) = full rating     (climb)
%       seg.thrust           "military" (default) | "max_continuous" |
%                            "drag" | "idle"                    (fixed_thrust)
%       seg.throttle         fraction of the rating, default 1  (fixed_thrust)
%       seg.config           polar for thrust = "drag", default "clean"
%       seg.drag_factor      thrust / level-flight drag, default 1 (fixed_thrust)
%       seg.speed_over_stall V / V_stall in seg.config (instead of a speed;
%                            follows the weight and wing)        (fixed_thrust)
%       seg.fuel_flow_pph    total fuel flow [lb/hr]; overrides the thrust
%                            model (needed for "idle")          (fixed_thrust)
%   A loiter with neither ktas nor mach is flown at MAXIMUM-ENDURANCE speed.
%
%   WHAT CHANGED FROM THE TTPA VERSION
%   The TTPA segments carried ktas only. A jet cruises at a MACH number
%   (cruise speed is set by compressibility, not by a speed in knots) and
%   climbs on an EQUIVALENT-airspeed schedule, so mach and keas were added.
%   weight_fraction was added so the fixed-fraction segments READ their
%   fraction from the requirements file instead of hardcoding it, which the
%   TTPA run_mission_L2 did (0.984, 0.992).
%
%   Keys a segment does not carry come back as [] (the same convention as
%   the TTPA code), so the segment functions can test isempty.

    s = miss.segments(seg_no);

    seg.type            = string(s.type);
    seg.name            = string(opt_(s, 'name', ""));
    seg.alt             = s.alt_ft;
    seg.mach            = opt_(s, 'mach',            []);
    seg.ktas            = opt_(s, 'ktas',            []);
    seg.keas            = opt_(s, 'keas',            []);
    seg.time_min        = opt_(s, 'time_min',        []);
    seg.dist            = opt_(s, 'distance_nm',     []);
    seg.weight_fraction = opt_(s, 'weight_fraction', []);
    seg.max_continuous  = logical(opt_(s, 'max_continuous', false));
    % fixed_thrust segments (see segment_fixed_thrust_jet)
    seg.thrust          = lower(string(opt_(s, 'thrust',      "military")));
    seg.throttle        = opt_(s, 'throttle',       1.0);
    seg.config          = string(opt_(s, 'config',  "clean"));
    seg.drag_factor     = opt_(s, 'drag_factor',    1.0);
    seg.fuel_flow_pph   = opt_(s, 'fuel_flow_pph',  []);
    seg.speed_over_stall = opt_(s, 'speed_over_stall', []);
end

function v = opt_(s, name, default)
%OPT_  s.(name) if present and non-empty, else the default.
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
