function [W_out, fuel_used, WF, info] = segment_cruise_jet(W_in, obj, seg_no)
%SEGMENT_CRUISE_JET  Jet cruise: Breguet range at the actual lift coefficient.
%
%   [W_out, fuel_used, WF, info] = segment_cruise_jet(W_in, obj, seg_no)
%
%   REPLACES segment_cruise_L2. The propeller Breguet equation is
%       R = 375 (eta_p / c_bhp) (L/D) ln(W_i/W_f)      [mi]
%   with the efficiency and BSFC in front. For a jet the fuel is burned per
%   pound of THRUST, so the speed appears explicitly
%   [Raymer Eq. 6.11; Roskam Part I Eq. 2.10]:
%
%       R = (V / c) (L/D) ln(W_i / W_f)
%       ->  W_f / W_i = exp( - R c / (V L/D) )
%
%   with c = TSFC [1/hr] (converted to 1/s here), V [ft/s], R [ft].
%
%   CRUISE AT A GIVEN MACH AND ALTITUDE. The requirement fixes Mach and
%   altitude, so q is fixed and the lift coefficient is pinned by the weight,
%   CL = W/(qS). As fuel burns off, CL falls, so the range is split into
%   n_sub pieces, each flown at its own CL (and therefore its own wave drag,
%   since the Korn drag-divergence Mach depends on CL). Same idea as the
%   TTPA L2 cruise.
%
%   Note: the best-range CL for a jet is sqrt(CD0/(3K)) (maximum of
%   sqrt(CL)/CD), not the best-L/D CL sqrt(CD0/K). The wing loading the
%   constraint analysis picks decides how close the cruise gets to it.
%
%   THRUST CHECK. If the drag at the first (heaviest) piece exceeds the max-
%   continuous thrust available at the cruise condition, a warning is raised:
%   the airplane cannot actually hold this Mach at this altitude. The cruise
%   constraint in the diagram is what prevents that at the design point.

    seg   = get_miss_seg(seg_no, obj.miss);
    if isempty(seg.mach)
        error('segment_cruise_jet:NoMach', 'Cruise segment %d has no mach.', seg_no);
    end
    state = get_state(seg.alt, seg.mach);
    V     = state.V;                             % ft/s
    q     = state.q;                             % lbf/ft^2
    S     = obj.geom.S_ref;                      % THIS pass's wing
    c_s   = obj.prop.tsfc(state) / 3600;         % 1/s

    n_sub = obj.miss.cruise_segments;
    dR    = seg.dist * 6076.115 / n_sub;          % ft per piece

    W = W_in;  CL_sum = 0;  LD_sum = 0;
    for i = 1:n_sub
        CL = W / (q * S);
        CD = obj.aero.CD(state, CL);
        LD = CL / CD;

        if i == 1
            T_avail = obj.prop.T_SL * obj.prop.thrust_lapse(state, "max_continuous");
            D       = q * S * CD;
            if D > T_avail
                warning('segment_cruise_jet:InsufficientThrust', ...
                    ['Cruise drag %.0f lbf exceeds the max-continuous thrust %.0f lbf ', ...
                     'at M %.2f, %.0f ft.'], D, T_avail, seg.mach, seg.alt);
            end
        end

        W = W * exp(-dR * c_s / (V * LD));
        CL_sum = CL_sum + CL;  LD_sum = LD_sum + LD;
    end

    WF        = W / W_in;
    fuel_used = W_in - W;
    W_out     = W;
    info = struct('type', "cruise", 'CL', CL_sum/n_sub, 'LD', LD_sum/n_sub, ...
                  'V_fps', V, 'time_hr', seg.dist * 6076.115 / V / 3600, ...
                  'n_sub', n_sub, 'dist_nm', seg.dist);
end
