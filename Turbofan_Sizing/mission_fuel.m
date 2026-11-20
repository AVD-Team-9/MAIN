function [W_fuel, fuel_fraction, fuel_burned, segment_weight, segment_wf, detail, summary] = mission_fuel(W_TO, obj)
%MISSION_FUEL  Design fuel weight: every sizing mission is flown, the hungriest one governs.
%
%   [W_fuel, fuel_fraction, fuel_burned, segment_weight, segment_wf, detail, summary] ...
%       = mission_fuel(W_TO, obj)
%
%   Inputs
%     W_TO   takeoff weight [lbf]
%     obj    discipline bundle; obj.missions is a cell array of mission
%            profiles (tf_disciplines), obj.miss the first of them
%   Outputs (for the GOVERNING mission - the one that needs the most fuel)
%     W_fuel          fuel to carry = burned x (1 + reserve_fuel_fraction) [lbf]
%     fuel_fraction   W_fuel / W_TO
%     fuel_burned     1xN fuel per segment [lbf]
%     segment_weight  1x(N+1) weight at each segment boundary [lbf]
%     segment_wf      1xN segment weight fractions
%     detail          1xN per-segment record (run_mission_jet)
%     summary         struct: names (1xM), W_fuel (1xM), governing (index),
%                     profiles (cell, 1xM), and per-mission fuel_burned/detail
%
%   MORE THAN ONE MISSION. The PRM2 RFP has two design missions (training and
%   ferry) and requires ALL fuel for BOTH to be carried internally
%   (RFP-3-P06-R01.1). So the airplane must carry the fuel of whichever needs
%   more: every mission in obj.missions is flown at the same W_TO and the
%   largest fuel wins. That mission is "governing", and it is what the
%   sizing loop closes on. A file with only std_mission behaves exactly as
%   before.
%
%   Each mission carries its own reserve_fuel_fraction (applied to its burned
%   fuel) and its own climb_segments / cruise_segments.

    arguments
        W_TO (1,1) double {mustBePositive}
        obj  (1,1) struct
    end

    if isfield(obj, 'missions') && ~isempty(obj.missions)
        profiles = obj.missions;
    else
        profiles = {obj.miss};
    end

    M = numel(profiles);
    summary = struct('names', strings(1, M), 'W_fuel', zeros(1, M), 'governing', 1, ...
                     'profiles', {profiles}, 'fuel_burned', {cell(1, M)}, ...
                     'detail', {cell(1, M)}, 'segment_weight', {cell(1, M)}, ...
                     'segment_wf', {cell(1, M)});

    for m = 1:M
        o = obj;
        o.miss = profiles{m};
        [fb, sw, swf, det] = run_mission_jet(W_TO, o);
        if isfield(o.miss, 'reserve_fuel_fraction') && ~isempty(o.miss.reserve_fuel_fraction)
            f_reserve = o.miss.reserve_fuel_fraction;
        else
            error('mission_fuel:MissingReserveFraction', ...
                'Mission profile "%s" has no reserve_fuel_fraction.', string(o.miss.name));
        end
        summary.names(m)          = string(o.miss.name);
        summary.W_fuel(m)         = sum(fb) * (1 + f_reserve);
        summary.fuel_burned{m}    = fb;
        summary.detail{m}         = det;
        summary.segment_weight{m} = sw;
        summary.segment_wf{m}     = swf;
    end

    [W_fuel, g] = max(summary.W_fuel);
    summary.governing = g;
    fuel_burned    = summary.fuel_burned{g};
    segment_weight = summary.segment_weight{g};
    segment_wf     = summary.segment_wf{g};
    detail         = summary.detail{g};
    fuel_fraction  = W_fuel / W_TO;
end
