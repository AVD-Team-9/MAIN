classdef TfGeom < GeometryBase
%TFGEOM  Level-2 geometry of a twin-turbofan airplane.
%
%   geom = TfGeom(json_path, prop)  reads the geometry block; then set
%   geom.S_ref (the sizing loop does this every pass).
%
%   Built from TtpaGeom. The wing, tail, fuselage and wetted-area equations
%   are the SAME (Raymer Eqs. 6.28-6.29, 7.6-7.8, 7.11-7.13), because they do
%   not care what drives the airplane. Three things changed:
%
%   1. SWEEP IS USED. The TTPA was unswept and only carried sweep for
%      reporting. A jet wing is swept to delay compressibility drag, and
%      the sweep enters two places downstream: the leading-edge sweep picks
%      the Oswald-efficiency formula in TfAero (Raymer Eq. 12.48 vs 12.49),
%      and the quarter-chord sweep enters the Korn drag-divergence equation.
%      Lambda_LE is therefore a Dependent property here.
%
%   2. THE NACELLE DIAMETER FOLLOWS THE ENGINE. TtpaGeom held a fixed 2.8 ft
%      nacelle because a piston engine's width hardly changes with power.
%      A turbofan's fan diameter grows as sqrt(thrust), so d_nacelle is now
%      Dependent on prop.T_SL through TfProp.engine_diameter, and the
%      nacelle wetted area grows faster with the engine than before.
%
%   3. THE NACELLE FACTORS ARE BOTH LAYOUT RATIOS: nacelle diameter / engine
%      diameter and nacelle length / engine length.
%
%   4. V-TAIL OPTION (geometry.tail_type = "v_tail"). See the V-TAIL
%      section below and V_TAIL_MODEL.md for the equations. The two
%      required tail areas still come from the volume coefficients; the V
%      is sized so its projections supply them, and S_ht / S_vt then
%      return the V's PROJECTED areas, so the wetted areas, the weights and
%      every printout keep working unchanged.
%
%   5. TWIN CANTED FINS (geometry.tail_type = "twin_vertical"). A normal
%      horizontal tail plus n_fins vertical fins canted cant_deg outboard.
%      The fins are sized so their VERTICAL projection supplies the yaw
%      requirement; S_vt is then the fins' true (panel) area. See the
%      TWIN VERTICAL TAILS section below and TWIN_TAIL_MODEL.md.
%
%   S_ref is the ONLY mutable property; everything else is Dependent and is
%   recomputed on every read, so nothing goes stale when the loop writes a
%   new wing area or a new thrust.

    properties
        S_ref = NaN
        % Wing reference area [ft^2]. THE design variable: written every pass
        % as W_TO / (W/S). NaN until the loop, or the caller, sets it.
    end

    properties (SetAccess = immutable)
        prop                        % injected TfProp (engine size -> nacelle)

        % --- wing shape ---
        AR                          % <- geometry.AR
        taper                       % <- geometry.taper_ratio
        sweep_qc_deg                % <- geometry.sweep_quarter_chord_deg
        tc_root                     % <- geometry.thickness_ratio_root
        tc_tip                      % <- geometry.thickness_ratio_tip
        twist_deg                   % reporting only
        incidence_deg               % reporting only
        dihedral_deg                % reporting only

        % --- tail arrangement ---
        tail_type = "conventional"  % <- geometry.tail_type: "conventional" | "v_tail"
        vtail = struct()            % <- geometry.v_tail (V-tail only, see constructor)
        twin  = struct()            % <- extra vertical_tail keys (twin_vertical only)

        % --- horizontal tail ---  (V-tail: the PITCH requirement of the V)
        V_ht                        % volume coefficient [-]
        L_ht                        % moment arm [ft]
        AR_ht
        taper_ht
        tc_ht
        sweep_qc_ht_deg

        % --- vertical tail ---  (V-tail: the YAW requirement of the V)
        V_vt
        L_vt
        AR_vt
        taper_vt
        tc_vt
        sweep_qc_vt_deg

        % --- fuselage (cabin-driven, constant) ---
        w_fus
        h_fus
        l_nose
        l_cabin
        l_cone
        w_cone_end
        h_cone_end

        % --- nacelle layout ratios ---
        k_nacelle_diameter          % nacelle diameter / bare engine diameter
        k_nacelle_length            % nacelle length   / bare engine length
        engine_installation         % "pod" (default) or "buried"  (NEW, optional)
                                    % <- geometry.nacelle.installation

        % --- landing-gear static-load ratios (post-sizing check only) ---
        l_n_over_l_d
        l_m_over_l_d
    end

    properties (Dependent)
        % --- wing planform ---
        b                           % span [ft]
        c_root                      % root chord [ft]
        c_tip                       % tip chord [ft]
        MAC                         % mean aerodynamic chord [ft]
        y_MAC                       % spanwise MAC station [ft]
        tc_mean                     % mean thickness ratio [-]
        sweep_LE_deg                % leading-edge sweep [deg]  (NEW)
        S_exposed_wing              % planform outside the fuselage [ft^2]

        % --- tails ---
        S_ht                        % horizontal tail area; V-tail: projected S_V cos^2(G)
        b_ht                        % V-tail: projected (tip-to-tip horizontal) span
        MAC_ht
        S_vt                        % vertical tail area;  V-tail: projected S_V sin^2(G)
        b_vt                        % V-tail: height of one panel
        MAC_vt
        S_ht_required               % volume-coefficient requirement (Raymer 6.29)
        S_vt_required               % volume-coefficient requirement (Raymer 6.28)

        % --- V-tail (NaN for a conventional tail) ---
        S_vtail                     % total panel area, both panels [ft^2]
        vtail_dihedral_deg          % panel dihedral from horizontal [deg]
        b_vtail                     % developed span, tip to tip along the panels [ft]
        b_vtail_projected           % horizontal projection of b_vtail [ft]
        MAC_vtail                   % panel MAC [ft]

        trim_tail_span              % span that carries the pitch-trim load (TfAero trim drag)

        % --- twin canted fins (NaN unless tail_type = "twin_vertical") ---
        n_fins                      % number of vertical fins (1 for other tail types)
        fin_cant_deg                % cant from vertical, outboard positive [deg]
        S_fin                       % true area of ONE fin [ft^2]
        S_vt_projected              % vertical projection of all fins, S_vt cos^2(cant)
        S_fin_pitch_projected       % horizontal projection of all fins, S_vt sin^2(cant)

        % --- fuselage ---
        l_fus
        fineness_ratio
        S_wet_fuselage

        % --- nacelles ---
        l_engine                    % bare length of one engine [ft]
        d_engine                    % bare diameter of one engine [ft] (NEW)
        l_nacelle                   % one nacelle [ft]
        d_nacelle                   % one nacelle [ft] - now Dependent (NEW)
        S_wet_nacelle               % ALL nacelles [ft^2]

        % --- wetted areas ---
        S_wet_wing
        S_wet_ht
        S_wet_vt
        S_wet
        S_wet_over_S_ref            % drives CD0 (Raymer Eq. 12.23)
    end

    methods

        %% Constructor
        function obj = TfGeom(json_path, prop)
            arguments
                json_path {mustBeTextScalar, mustBeNonzeroLengthText}
                prop (1,1) ThrustPropulsionBase
            end

            obj.prop = prop;

            J = jsondecode(fileread(json_path));
            G = J.geometry;

            obj.AR            = G.AR;
            obj.taper         = G.taper_ratio;
            obj.sweep_qc_deg  = G.sweep_quarter_chord_deg;
            obj.tc_root       = G.thickness_ratio_root;
            obj.tc_tip        = G.thickness_ratio_tip;
            obj.twist_deg     = G.twist_deg;
            obj.incidence_deg = G.incidence_deg;
            obj.dihedral_deg  = G.dihedral_deg;

            if isfield(G, 'tail_type') && ~isempty(G.tail_type)
                obj.tail_type = lower(string(G.tail_type));
            end
            switch obj.tail_type
                case "conventional"
                    H = G.horizontal_tail;
                    obj.V_ht = H.volume_coefficient;   obj.L_ht  = H.arm_ft;
                    obj.AR_ht = H.AR;                  obj.taper_ht = H.taper_ratio;
                    obj.tc_ht = H.thickness_ratio;     obj.sweep_qc_ht_deg = H.sweep_quarter_chord_deg;

                    V = G.vertical_tail;
                    obj.V_vt = V.volume_coefficient;   obj.L_vt  = V.arm_ft;
                    obj.AR_vt = V.AR;                  obj.taper_vt = V.taper_ratio;
                    obj.tc_vt = V.thickness_ratio;     obj.sweep_qc_vt_deg = V.sweep_quarter_chord_deg;

                case "v_tail"
                    % geometry.v_tail block (both panels together):
                    %   horizontal_volume_coefficient  c_HT the V must supply in pitch
                    %   vertical_volume_coefficient    c_VT the V must supply in yaw
                    %   arm_ft                         wing MAC/4 to tail MAC/4 [ft]
                    %   sizing                         "fixed_dihedral" | "projected_area"
                    %   dihedral_deg                   panel dihedral (fixed_dihedral only)
                    %   AR                             DEVELOPED aspect ratio b_V^2/S_V,
                    %                                  b_V tip to tip along the panels
                    %   taper_ratio, thickness_ratio, sweep_quarter_chord_deg   of a panel
                    %   fuselage_width_at_tail_ft      (optional) carry-through width for
                    %                                  the tail weight; default = the
                    %                                  tailcone end width
                    if ~isfield(G, 'v_tail')
                        error('TfGeom:NoVTailBlock', 'tail_type = "v_tail" needs a geometry.v_tail block.');
                    end
                    T = G.v_tail;
                    if ~isfield(T, 'sizing') || isempty(T.sizing), T.sizing = "fixed_dihedral"; end
                    T.sizing = lower(string(T.sizing));
                    if ~ismember(T.sizing, ["fixed_dihedral", "projected_area"])
                        error('TfGeom:BadVTailSizing', ...
                            'v_tail.sizing must be "fixed_dihedral" or "projected_area", not "%s".', T.sizing);
                    end
                    if T.sizing == "fixed_dihedral" && ~(isfield(T, 'dihedral_deg') ...
                            && T.dihedral_deg > 0 && T.dihedral_deg < 90)
                        error('TfGeom:BadVTailDihedral', ...
                            'v_tail.sizing = "fixed_dihedral" needs 0 < dihedral_deg < 90.');
                    end
                    if ~isfield(T, 'fuselage_width_at_tail_ft') || isempty(T.fuselage_width_at_tail_ft)
                        T.fuselage_width_at_tail_ft = G.fuselage.tailcone_end_width_ft;
                    end
                    obj.vtail = T;
                    obj.V_ht = T.horizontal_volume_coefficient;   obj.L_ht = T.arm_ft;
                    obj.V_vt = T.vertical_volume_coefficient;     obj.L_vt = T.arm_ft;
                    % Both "tails" ARE the V's panels: same section, taper and sweep.
                    obj.AR_ht = T.AR;  obj.taper_ht = T.taper_ratio;
                    obj.tc_ht = T.thickness_ratio;  obj.sweep_qc_ht_deg = T.sweep_quarter_chord_deg;
                    obj.AR_vt = T.AR;  obj.taper_vt = T.taper_ratio;
                    obj.tc_vt = T.thickness_ratio;  obj.sweep_qc_vt_deg = T.sweep_quarter_chord_deg;

                case "twin_vertical"
                    % horizontal_tail block as for a conventional tail.
                    % vertical_tail block as for a conventional tail, with
                    %   volume_coefficient  the EFFECTIVE (yaw) c_VT, i.e. on the
                    %                       fins' vertical projection
                    %   AR, taper_ratio ... of ONE fin (AR = h_fin^2 / S_fin,
                    %                       h_fin measured along the fin)
                    % plus
                    %   n_fins              default 2
                    %   cant_deg            fin cant from vertical, outboard + [deg]
                    %                       (default 0)
                    %   credit_fins_in_pitch  (default false) let the fins'
                    %                       horizontal projection count toward
                    %                       the horizontal-tail requirement
                    H = G.horizontal_tail;
                    obj.V_ht = H.volume_coefficient;   obj.L_ht  = H.arm_ft;
                    obj.AR_ht = H.AR;                  obj.taper_ht = H.taper_ratio;
                    obj.tc_ht = H.thickness_ratio;     obj.sweep_qc_ht_deg = H.sweep_quarter_chord_deg;

                    V = G.vertical_tail;
                    obj.V_vt = V.volume_coefficient;   obj.L_vt  = V.arm_ft;
                    obj.AR_vt = V.AR;                  obj.taper_vt = V.taper_ratio;
                    obj.tc_vt = V.thickness_ratio;     obj.sweep_qc_vt_deg = V.sweep_quarter_chord_deg;

                    d = struct('n_fins', 2, 'cant_deg', 0, 'credit_fins_in_pitch', false);
                    T = struct();
                    for f = string(fieldnames(d))'
                        if isfield(V, f) && ~isempty(V.(f)), T.(f) = V.(f); else, T.(f) = d.(f); end
                    end
                    T.credit_fins_in_pitch = logical(T.credit_fins_in_pitch);
                    if ~(T.n_fins >= 1 && mod(T.n_fins, 1) == 0)
                        error('TfGeom:BadFinCount', 'vertical_tail.n_fins must be a positive integer.');
                    end
                    if ~(T.cant_deg >= 0 && T.cant_deg < 60)
                        error('TfGeom:BadFinCant', 'vertical_tail.cant_deg must be in [0, 60) deg.');
                    end
                    obj.twin = T;

                otherwise
                    error('TfGeom:UndefinedTailType', ...
                        'geometry.tail_type must be "conventional", "v_tail" or "twin_vertical", not "%s".', obj.tail_type);
            end

            F = G.fuselage;
            obj.w_fus      = F.width_ft;
            obj.h_fus      = F.height_ft;
            obj.l_nose     = F.nose_length_ft;
            obj.l_cabin    = F.constant_length_ft;
            obj.l_cone     = F.tailcone_length_ft;
            obj.w_cone_end = F.tailcone_end_width_ft;
            obj.h_cone_end = F.tailcone_end_height_ft;

            N = G.nacelle;
            obj.k_nacelle_diameter = N.diameter_over_engine_diameter;
            obj.k_nacelle_length   = N.length_over_engine_length;
            % Optional: where the engines are. "pod" = podded nacelles (on
            % pylons or on the rear fuselage) that add their own wetted area.
            % "buried" = engines inside the fuselage (fighters, trainers such
            % as the T-45): there is no separate nacelle, the intakes and the
            % engine bay are already part of the fuselage wetted area, so the
            % nacelle wetted area is ZERO. Older requirement files without
            % the key keep the podded behaviour.
            obj.engine_installation = "pod";
            if isfield(N, 'installation') && ~isempty(N.installation)
                obj.engine_installation = string(N.installation);
            end
            if ~ismember(obj.engine_installation, ["pod", "buried"])
                error('TfGeom:UndefinedInstallation', ...
                    'geometry.nacelle.installation must be "pod" or "buried", not "%s".', ...
                    obj.engine_installation);
            end

            L = G.landing_gear;
            obj.l_n_over_l_d = L.l_n_over_l_d;
            obj.l_m_over_l_d = L.l_m_over_l_d;
        end

        %% ---------------- GeometryBase abstract interface ----------------
        function val = get_S_ref(obj)
            val = obj.S_ref;
        end

        function val = get_S_wet(obj, ~)
            % The W_TO argument is only needed by a Level-1 (regression)
            % geometry; this one adds up real components.
            val = obj.S_wet;
        end

        %% ---------------- Wing planform ----------------
        function val = get.b(obj)
            val = GeometryBase.compute_span(obj.AR, obj.S_ref);          % AR = b^2/S
        end

        function val = get.c_root(obj)
            val = GeometryBase.compute_root_chord(obj.S_ref, obj.b, obj.taper);   % Raymer 7.6
        end

        function val = get.c_tip(obj)
            val = GeometryBase.compute_tip_chord(obj.c_root, obj.taper);          % Raymer 7.7
        end

        function val = get.MAC(obj)
            val = GeometryBase.compute_mac(obj.c_root, obj.taper);                % Raymer 7.8
        end

        function val = get.y_MAC(obj)
            val = TfGeom.compute_y_mac(obj.b, obj.taper);
        end

        function val = get.tc_mean(obj)
            val = 0.5 * (obj.tc_root + obj.tc_tip);
        end

        function val = get.sweep_LE_deg(obj)
        %GET.SWEEP_LE_DEG  Leading-edge sweep from the quarter-chord sweep.
        %
        %   For a straight-tapered, mirrored wing the sweep of the line at
        %   chord fraction x satisfies
        %       tan(L_x) = tan(L_LE) - (4/AR) x (1 - lambda)/(1 + lambda)
        %   (the identity GeometryBase.convert_sweep implements, going LE ->
        %   x). Solving it the other way with x = 0.25:
        %       tan(L_LE) = tan(L_qc) + (1/AR) (1 - lambda)/(1 + lambda)
        %   A tapered wing's leading edge is always swept MORE than its
        %   quarter chord.
            t = tand(obj.sweep_qc_deg) + (1/obj.AR) * (1 - obj.taper) / (1 + obj.taper);
            val = atand(t);
        end

        function val = get.S_exposed_wing(obj)
        %GET.S_EXPOSED_WING  Planform outside the fuselage [ft^2].
        %   Same exact straight-taper integral as TtpaGeom:
        %       S_covered = c_root w_fus (1 - (1 - lambda) w_fus/(2 b))
            S_covered = obj.c_root * obj.w_fus * (1 - (1 - obj.taper) * obj.w_fus / (2 * obj.b));
            val = obj.S_ref - S_covered;
        end

        %% ---------------- Tails (volume-coefficient method) ----------------
        function val = get.S_ht_required(obj)
            % Raymer Eq. 6.29:  S_ht = c_HT * S_ref * MAC / L_ht
            val = obj.V_ht * obj.S_ref * obj.MAC / obj.L_ht;
        end

        function val = get.S_vt_required(obj)
            % Raymer Eq. 6.28:  S_vt = c_VT * S_ref * b / L_vt   (SPAN, not MAC)
            val = obj.V_vt * obj.S_ref * obj.b / obj.L_vt;
        end

        function val = get.S_ht(obj)
            if obj.tail_type == "v_tail"
                val = obj.S_vtail * cosd(obj.vtail_dihedral_deg)^2;   % projected (pitch)
            elseif obj.tail_type == "twin_vertical" && obj.twin.credit_fins_in_pitch
                val = max(obj.S_ht_required - obj.S_fin_pitch_projected, 0);
            else
                val = obj.S_ht_required;
            end
        end

        function val = get.b_ht(obj)
            if obj.tail_type == "v_tail"
                val = obj.b_vtail_projected;
            else
                val = GeometryBase.compute_span(obj.AR_ht, obj.S_ht);
            end
        end

        function val = get.MAC_ht(obj)
            if obj.tail_type == "v_tail"
                val = obj.MAC_vtail;  return
            end
            cr  = GeometryBase.compute_root_chord(obj.S_ht, obj.b_ht, obj.taper_ht);
            val = GeometryBase.compute_mac(cr, obj.taper_ht);
        end

        function val = get.S_vt(obj)
            if obj.tail_type == "v_tail"
                val = obj.S_vtail * sind(obj.vtail_dihedral_deg)^2;   % projected (yaw)
            elseif obj.tail_type == "twin_vertical"
                % TRUE area of all fins, sized so the vertical projection
                % meets the requirement: S_vt cos^2(cant) = S_v,req
                val = obj.S_vt_required / cosd(obj.twin.cant_deg)^2;
            else
                val = obj.S_vt_required;
            end
        end

        function val = get.b_vt(obj)
            if obj.tail_type == "v_tail"
                val = 0.5 * obj.b_vtail * sind(obj.vtail_dihedral_deg);   % one panel's height
            elseif obj.tail_type == "twin_vertical"
                % span of ONE fin, measured along the fin: AR_fin = h^2 / S_fin
                val = GeometryBase.compute_span(obj.AR_vt, obj.S_fin);
            else
                % Single-panel surface: AR_vt = b_vt^2 / S_vt
                val = GeometryBase.compute_span(obj.AR_vt, obj.S_vt);
            end
        end

        function val = get.MAC_vt(obj)
            if obj.tail_type == "v_tail"
                val = obj.MAC_vtail;  return
            end
            if obj.tail_type == "twin_vertical"      % one fin's MAC
                cr  = GeometryBase.compute_root_chord(obj.S_fin, obj.b_vt, obj.taper_vt);
                val = GeometryBase.compute_mac(cr, obj.taper_vt);  return
            end
            cr  = GeometryBase.compute_root_chord(obj.S_vt, obj.b_vt, obj.taper_vt);
            val = GeometryBase.compute_mac(cr, obj.taper_vt);
        end

        %% ---------------- Twin vertical tails ----------------
        %   A fin canted phi outboard of vertical sees a sideslip beta as a
        %   normal flow angle beta cos(phi), and only cos(phi) of its normal
        %   force is a side force - the same projection argument as the V-tail
        %   (Purser & Campbell, NACA Rep. 823), with Gamma = 90 deg - phi:
        %
        %       yaw (effective vertical) area   = S_vt cos^2(phi)
        %       pitch (horizontal) projection   = S_vt sin^2(phi)
        %
        %   So the fins' TRUE area is S_vt = S_v,req / cos^2(phi), where
        %   S_v,req = c_VT S b / L_vt with c_VT the EFFECTIVE coefficient.
        %   Each fin is S_fin = S_vt / n_fins, with span h = sqrt(AR S_fin)
        %   along the fin. Wetted area and weight use the true area.
        %
        %   ASSUMPTIONS: no fin-to-fin or fin-fuselage end-plate interference
        %   (closely spaced fins on a fuselage boom can gain or lose
        %   effectiveness); no fuselage/wing wake shielding - the reason twin
        %   fins are chosen for high-alpha directional stability is NOT
        %   credited; the pitch projection is ignored unless
        %   credit_fins_in_pitch = true.
        function val = get.n_fins(obj)
            if obj.tail_type == "twin_vertical", val = obj.twin.n_fins; else, val = 1; end
        end

        function val = get.fin_cant_deg(obj)
            if obj.tail_type == "twin_vertical", val = obj.twin.cant_deg; else, val = NaN; end
        end

        function val = get.S_fin(obj)
            if obj.tail_type == "twin_vertical", val = obj.S_vt / obj.twin.n_fins; else, val = NaN; end
        end

        function val = get.S_vt_projected(obj)
            if obj.tail_type == "twin_vertical"
                val = obj.S_vt * cosd(obj.twin.cant_deg)^2;
            else
                val = NaN;
            end
        end

        function val = get.S_fin_pitch_projected(obj)
            if obj.tail_type == "twin_vertical"
                val = obj.S_vt * sind(obj.twin.cant_deg)^2;
            else
                val = NaN;
            end
        end

        %% ---------------- V-tail ----------------
        %   [Purser & Campbell, NACA Report 823 (1945); Raymer Sec. 4.5]
        %
        %   A V-tail panel pair of total area S_V at dihedral G acts, to first
        %   order, like a horizontal tail of area S_V cos^2(G) in pitch and a
        %   vertical tail of area S_V sin^2(G) in yaw: a panel's normal force
        %   is proportional to the flow angle normal to it (cos G of an angle-
        %   of-attack change, sin G of a sideslip), and only the cos G (or
        %   sin G) component of that force acts in the plane of interest.
        %   The two requirements from the volume coefficients therefore read
        %
        %       S_V cos^2 G >= S_h,req      S_V sin^2 G >= S_v,req
        %
        %   sizing = "projected_area": both hold with equality, which gives the
        %   classic rule
        %       S_V = S_h,req + S_v,req,     G = atan( sqrt(S_v,req / S_h,req) )
        %   sizing = "fixed_dihedral": G is given (e.g. from the drawn layout)
        %   and the V is the smallest that meets both requirements:
        %       S_V = max( S_h,req / cos^2 G , S_v,req / sin^2 G )
        %   With volume coefficients backed out of a drawn V (as in the
        %   concept-5 files) both rules give the drawn area and dihedral.
        %
        %   ASSUMPTIONS: the cos^2/sin^2 projections ignore the panels'
        %   lower lift-curve slope at low aspect ratio, the panel-to-panel
        %   interference and fuselage sidewash. Wind-tunnel work after NACA
        %   823 generally found real V-tails need somewhat MORE area than the
        %   rule; carry that margin by raising the volume coefficients.
        function val = get.S_vtail(obj)
            if obj.tail_type ~= "v_tail", val = NaN; return, end
            Sh = obj.S_ht_required;  Sv = obj.S_vt_required;
            if obj.vtail.sizing == "projected_area"
                val = Sh + Sv;
            else
                G   = obj.vtail.dihedral_deg;
                val = max(Sh / cosd(G)^2, Sv / sind(G)^2);
            end
        end

        function val = get.vtail_dihedral_deg(obj)
            if obj.tail_type ~= "v_tail", val = NaN; return, end
            if obj.vtail.sizing == "projected_area"
                val = atand(sqrt(obj.S_vt_required / obj.S_ht_required));
            else
                val = obj.vtail.dihedral_deg;
            end
        end

        function val = get.b_vtail(obj)
            % Developed span, tip to tip ALONG the panels: AR_V = b_V^2 / S_V
            if obj.tail_type ~= "v_tail", val = NaN; return, end
            val = GeometryBase.compute_span(obj.vtail.AR, obj.S_vtail);
        end

        function val = get.b_vtail_projected(obj)
            if obj.tail_type ~= "v_tail", val = NaN; return, end
            val = obj.b_vtail * cosd(obj.vtail_dihedral_deg);
        end

        function val = get.MAC_vtail(obj)
            if obj.tail_type ~= "v_tail", val = NaN; return, end
            cr  = GeometryBase.compute_root_chord(obj.S_vtail, obj.b_vtail, obj.vtail.taper_ratio);
            val = GeometryBase.compute_mac(cr, obj.vtail.taper_ratio);
        end

        function val = get.trim_tail_span(obj)
            % The pitch-trim load is a VERTICAL force; its induced drag is set
            % by the horizontal projection of the lifting system's span (Munk's
            % theorem), so the V uses b_V cos G and a conventional tail its span.
            if obj.tail_type == "v_tail"
                val = obj.b_vtail_projected;
            else
                val = obj.b_ht;
            end
        end

        %% ---------------- Fuselage (constant) ----------------
        function val = get.l_fus(obj)
            val = obj.l_nose + obj.l_cabin + obj.l_cone;
        end

        function val = get.fineness_ratio(obj)
            val = obj.l_fus / max(obj.w_fus, obj.h_fus);
        end

        function val = get.S_wet_fuselage(obj)
        %GET.S_WET_FUSELAGE  [Raymer Eq. 7.13]  S_wet ~ 3.4 (A_top + A_side)/2
        %   Nose and tail cone taken as linear tapers, the cabin as a
        %   constant section - the same three-piece model as TtpaGeom.
            A_top  = 0.5 * obj.w_fus * obj.l_nose + obj.w_fus * obj.l_cabin ...
                   + 0.5 * (obj.w_fus + obj.w_cone_end) * obj.l_cone;
            A_side = 0.5 * obj.h_fus * obj.l_nose + obj.h_fus * obj.l_cabin ...
                   + 0.5 * (obj.h_fus + obj.h_cone_end) * obj.l_cone;
            val = 3.4 * (A_top + A_side) / 2;
        end

        %% ---------------- Nacelles (follow the engine) ----------------
        function val = get.l_engine(obj)
            val = obj.prop.engine_length();             % Raymer Eq. 10.2
        end

        function val = get.d_engine(obj)
            val = obj.prop.engine_diameter();           % Raymer Eq. 10.3
        end

        function val = get.l_nacelle(obj)
            val = obj.k_nacelle_length * obj.l_engine;
        end

        function val = get.d_nacelle(obj)
            val = obj.k_nacelle_diameter * obj.d_engine;
        end

        function val = get.S_wet_nacelle(obj)
        %GET.S_WET_NACELLE  Wetted area of ALL nacelles [ft^2].
        %   Each nacelle as a cylinder: pi * D * L. The inlet face and the
        %   nozzle exit are not wetted. The pylons are NOT included - add
        %   them as a small extra area if your layout has long pylons.
        %   A BURIED engine has no nacelle of its own (its skin is the
        %   fuselage's), so it adds nothing here; its size still enters the
        %   weights through TfProp.engine_weight.
            if obj.engine_installation == "buried"
                val = 0;
                return
            end
            val = obj.prop.n_engines * pi * obj.d_nacelle * obj.l_nacelle;
        end

        %% ---------------- Wetted areas ----------------
        function val = get.S_wet_wing(obj)
            val = TfGeom.wetted_from_exposed(obj.S_exposed_wing, obj.tc_mean);
        end

        % V-tail: S_ht + S_vt = S_V (cos^2 + sin^2 = 1) and both use the panel
        % t/c, so these two rows add up to the wetted area of the V, counted
        % once. The V's interference drag (two root junctions instead of a
        % conventional tail's three) is handled in TfAero.
        function val = get.S_wet_ht(obj)
            val = TfGeom.wetted_from_exposed(obj.S_ht, obj.tc_ht);
        end

        function val = get.S_wet_vt(obj)
            val = TfGeom.wetted_from_exposed(obj.S_vt, obj.tc_vt);
        end

        function val = get.S_wet(obj)
            val = obj.S_wet_wing + obj.S_wet_ht + obj.S_wet_vt ...
                + obj.S_wet_fuselage + obj.S_wet_nacelle;
        end

        function val = get.S_wet_over_S_ref(obj)
            val = obj.S_wet / obj.S_ref;
        end

        %% ---------------- Post-sizing check ----------------
        function V_ft3 = wing_fuel_volume(obj)
        %WING_FUEL_VOLUME  Usable wing fuel volume [ft^3], Torenbeek:
        %   V = 0.54 (S^2/b) (t/c)_root (1 + lambda sqrt(tau) + lambda^2 tau)/(1+lambda)^2
        %   with tau = (t/c)_tip/(t/c)_root. Same relation as TtpaGeom. A
        %   business jet often carries fuel in the fuselage too; this only
        %   checks the wing.
            tau   = obj.tc_tip / obj.tc_root;
            V_ft3 = 0.54 * (obj.S_ref^2 / obj.b) * obj.tc_root ...
                  * (1 + obj.taper * sqrt(tau) + obj.taper^2 * tau) / (1 + obj.taper)^2;
        end
    end

    methods (Static)
        function y = compute_y_mac(b, lambda)
        %COMPUTE_Y_MAC  Spanwise MAC station  y = b (1 + 2 lambda)/(6 (1 + lambda)).
            y = b * (1 + 2*lambda) / (6 * (1 + lambda));
        end

        function S_wet = wetted_from_exposed(S_exposed, tc)
        %WETTED_FROM_EXPOSED  [Raymer Eqs. 7.11, 7.12]
        %   t/c < 0.05:  S_wet = 2.003 S_exp;   else S_wet = S_exp (1.977 + 0.52 t/c)
            if tc < 0.05
                S_wet = 2.003 * S_exposed;
            else
                S_wet = S_exposed * (1.977 + 0.52 * tc);
            end
        end
    end
end
