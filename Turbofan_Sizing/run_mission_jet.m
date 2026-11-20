function [fuel_burned, segment_weight, segment_wf, detail] = run_mission_jet(W_TO, obj)
%RUN_MISSION_JET  The jet mission, segment by segment.
%
%   [fuel_burned, segment_weight, segment_wf, detail] = run_mission_jet(W_TO, obj)
%
%   Outputs
%     fuel_burned     1xN fuel burned in each segment [lbf]
%     segment_weight  1x(N+1) weight at the start of each segment, then the end
%     segment_wf      1xN weight fraction of each segment
%     detail          1xN struct: type, CL, L/D, speed, time, distance
%
%   Same dispatcher as the TTPA run_mission_L2: each segment's TYPE is read
%   from the mission profile, so a new mission is a change to the JSON only.
%
%       type       method
%       takeoff    fixed fraction, READ from the segment's weight_fraction
%       climb      segment_climb_jet   (excess thrust, EAS/Mach schedule)
%       cruise     segment_cruise_jet  (jet Breguet range, actual CL)
%       descent    fixed fraction, read
%       loiter     segment_loiter_jet  (jet Breguet endurance, actual CL)
%       landing    fixed fraction, read
%
%   CHANGE from the TTPA: the fixed fractions are no longer typed into this
%   file (the TTPA hardcoded 0.984 and 0.992). They come from each segment's
%   weight_fraction key, so they match the airplane class you put in the JSON
%   (the placeholder values are Roskam Part I Table 2.1, business jet).

    n = numel(obj.miss.segments);

    fuel_burned    = zeros(1, n);
    segment_wf     = zeros(1, n);
    segment_weight = zeros(1, n + 1);
    segment_weight(1) = W_TO;

    blank  = struct('type', "", 'CL', NaN, 'LD', NaN, 'V_fps', NaN, ...
                    'time_hr', NaN, 'n_sub', NaN, 'dist_nm', NaN);
    detail = repmat(blank, 1, n);

    W = W_TO;
    for k = 1:n
        seg  = get_miss_seg(k, obj.miss);
        info = blank;
        info.type = lower(seg.type);

        switch info.type
            case {"takeoff", "descent", "landing"}
                [W, fuel_burned(k), segment_wf(k)] = fixed_fraction_(W, seg, k);
            case "climb"
                [W, fuel_burned(k), segment_wf(k), info] = segment_climb_jet(W, obj, k);
            case "cruise"
                [W, fuel_burned(k), segment_wf(k), info] = segment_cruise_jet(W, obj, k);
            case "loiter"
                [W, fuel_burned(k), segment_wf(k), info] = segment_loiter_jet(W, obj, k);
            case "fixed_thrust"
                [W, fuel_burned(k), segment_wf(k), info] = segment_fixed_thrust_jet(W, obj, k);
            otherwise
                error('run_mission_jet:UndefinedSegment', ...
                    'Mission segment type "%s" is not defined.', info.type);
        end

        segment_weight(k + 1) = W;
        detail(k) = info;
    end
end


function [W_out, fuel_used, WF] = fixed_fraction_(W_in, seg, k)
%FIXED_FRACTION_  A segment flown at the historical weight fraction it carries.
    WF = seg.weight_fraction;
    if isempty(WF)
        error('run_mission_jet:NoWeightFraction', ...
            'Segment %d (%s) needs a weight_fraction key in the JSON.', k, seg.type);
    end
    fuel_used = (1 - WF) * W_in;
    W_out     = W_in * WF;
end
