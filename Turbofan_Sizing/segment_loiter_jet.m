function [W_out, fuel_used, WF, info] = segment_loiter_jet(W_in, obj, seg_no)
%SEGMENT_LOITER_JET  Jet loiter (hold): Breguet endurance at the actual CL.
%
%   [W_out, fuel_used, WF, info] = segment_loiter_jet(W_in, obj, seg_no)
%
%   REPLACES segment_loiter_L2. Propeller endurance carries V and eta_p,
%   because fuel flow follows POWER = drag x speed. A jet's fuel flow follows
%   THRUST = drag, so the speed drops out [Raymer Eq. 6.14; Roskam Part I
%   Eq. 2.12]:
%
%       E = (1/c) (L/D) ln(W_i / W_f)
%       ->  W_f / W_i = exp( - E c / (L/D) )      E [hr], c = TSFC [1/hr]
%
%   which is why a jet holds at best L/D (CL = sqrt(CD0/K)), while a
%   propeller airplane holds slower, at the maximum of CL^1.5/CD, where
%   L/D = 0.866 L/D_max (the factor the IHW1 loiter used).
%
%   The hold speed and altitude are given, so CL = W/(qS) is pinned, exactly
%   as in the TTPA loiter. One piece is enough: a 45-minute hold burns a
%   small share of the weight.

    seg = get_miss_seg(seg_no, obj.miss);
    s0  = get_state(seg.alt, 0);
    if ~isempty(seg.ktas)
        V = seg.ktas * 1.68781;                  % ft/s, as given
    elseif ~isempty(seg.mach)
        V = seg.mach * s0.a;
    else
        % MAXIMUM-ENDURANCE SPEED. A jet's endurance E = (1/c) (L/D) ln(Wi/Wf)
        % is largest at (L/D)max, i.e. at CL* = sqrt(CD0/K) [Raymer Sec. 17.3]
        % - the speed the RFP's reserve definitions ask for ("maximum
        % endurance speed in cruise configuration"). Flown at the START
        % weight; holding CL* as weight falls means slowing down, which is
        % what the constant-CL Breguet form below assumes anyway.
        CLs = obj.aero.CL_LDmax;
        V   = sqrt(2 * W_in / (s0.rho * obj.geom.S_ref * CLs));
    end
    st  = get_state(seg.alt, V / s0.a);          % state at the hold Mach

    q   = st.q;
    S   = obj.geom.S_ref;
    c   = obj.prop.tsfc(st);                     % 1/hr
    E   = seg.time_min / 60;                     % hr

    CL  = W_in / (q * S);
    CD  = obj.aero.CD(st, CL);
    LD  = CL / CD;

    WF        = exp(-E * c / LD);
    fuel_used = (1 - WF) * W_in;
    W_out     = W_in * WF;
    info = struct('type', "loiter", 'CL', CL, 'LD', LD, 'V_fps', V, ...
                  'time_hr', E, 'n_sub', 1, 'dist_nm', V * E * 3600 / 6076.115);
end
