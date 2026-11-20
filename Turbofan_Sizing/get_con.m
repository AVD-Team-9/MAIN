function [con] = get_con(con_no, cons)
%GET_CON  One constraint condition out of the constraint set (turbofan).
%
%   con = get_con(con_no, cons) pulls condition number con_no out of the
%   struct array ConstraintSetImporter.read_conditions returned, and fills
%   every optional key with its default so the constraint functions never
%   have to test isfield themselves.
%
%   Same idea as the TTPA get_con. The keys changed because a jet
%   constraint set is written differently:
%
%   REQUIREMENT keys (left EMPTY when the condition does not carry them):
%       distance_ft     field length [ft]               takeoff, landing
%       G               climb gradient [-]              climb_gradient
%       approach_speed_kts  maximum approach speed [KEAS]   approach_speed
%       distance_is_actual  false; true = landing distance_ft is an actual
%                           distance, not a FAR 25 field length   landing
%
%   OPTIONAL keys and their defaults:
%       config             "clean"   flap/gear configuration name
%       beta               1.0       W_condition / W_TO
%       throttle           1.0       throttle fraction of the rating
%       oei                false     one engine inoperative
%       max_continuous     false     max-continuous rating instead of takeoff
%       engine_out_drag    false     add the windmilling-engine drag
%                                    (replaces the TTPA propeller_stopped)
%       mach               0         flight Mach number (enters the lapse)
%       n                  1         load factor           (flight conditions)
%       Ps_fps             0         specific excess power (flight conditions)
%       speed_over_stall   1.2       V / V_stall that a climb is flown at
%                                    (FAR 25: V2 = 1.2, en-route 1.25,
%                                    balked landing 1.3, approach 1.5)
%
%   REMOVED from the TTPA version: ktas, power_index, power_setting,
%   propeller_stopped, method - all of them belonged to the propeller
%   formulation. 'throttle' replaces power_setting.

    c = cons(con_no);

    con.name = string(c.name);
    con.type = string(c.type);
    con.alt  = c.altitude_ft;

    % Requirement of this condition, whichever one applies
    con.distance_ft = get_field(c, 'distance_ft', []);
    con.G           = get_field(c, 'G',           []);
    con.approach_speed_kts = get_field(c, 'approach_speed_kts', []);
    % Landing only: true when distance_ft is an ACTUAL landing distance
    % (military, MIL-STD-3013B style) rather than a FAR 25 field length,
    % which already contains the 1/0.6 operational factor.
    con.distance_is_actual = logical(get_field(c, 'distance_is_actual', false));

    % Configuration, weight basis and thrust basis
    con.config           = string(get_field(c, 'config', "clean"));
    con.beta             = get_field(c, 'beta',     1.0);
    con.throttle         = get_field(c, 'throttle', 1.0);
    con.oei              = logical(get_field(c, 'oei',             false));
    con.max_continuous   = logical(get_field(c, 'max_continuous',  false));
    con.engine_out_drag  = logical(get_field(c, 'engine_out_drag', false));

    % Flight condition (the lapse needs Mach; the master equation needs n, Ps)
    con.mach             = get_field(c, 'mach',   0);
    con.n                = get_field(c, 'n',      1);
    con.Ps_fps           = get_field(c, 'Ps_fps', 0);
    con.speed_over_stall = get_field(c, 'speed_over_stall', 1.2);
end

%% Supporting Functions
function [val] = get_field(c, name, default)
%GET_FIELD  c.(name) if it is present and non-empty, otherwise the default.
%   json_as_struct_array gives EVERY condition EVERY key (the union of all
%   keys), with [] where a condition did not carry one, so "present" is
%   not enough - the value must also be non-empty.
    if isfield(c, name) && ~isempty(c.(name))
        val = c.(name);
    else
        val = default;
    end
end
