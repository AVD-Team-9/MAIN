classdef TfAero < AerodynamicsBase
%TFAERO  Level-2 aerodynamics of a swept-wing twin-turbofan airplane.
%
%   aero = TfAero(json_path, geom)
%
%   Built from TtpaAero. What stayed the same:
%     * CD0 is Dependent on the geometry:  CD0 = Cfe * S_wet/S_ref
%       [Raymer Eq. 12.23]. This is still the coupling that lets the sizing
%       loop's wing area reach the drag, the mission fuel and the weight.
%     * K = 1/(pi AR e), and the configuration polars (flap and gear
%       increments to CD0 and e, Roskam Part I Table 3.6).
%
%   What changed, and why:
%
%   1. OSWALD EFFICIENCY FOR A SWEPT WING. TtpaAero used only Raymer
%      Eq. 12.48, which is valid for a leading-edge sweep below about 30 deg.
%      Here the leading-edge sweep from the geometry picks the branch:
%        Lambda_LE <= 30 deg:  e = 1.78 (1 - 0.045 AR^0.68) - 0.64       [12.48]
%        Lambda_LE >  30 deg:  e = 4.61 (1 - 0.045 AR^0.68) cos(L_LE)^0.15 - 3.1  [12.49]
%      The two formulas do not meet exactly at 30 deg, so e jumps a little
%      if a design straddles 30 deg - keep that in mind when you sweep it.
%
%   2. COMPRESSIBILITY (WAVE) DRAG. A turbofan airplane cruises at M 0.7-0.85,
%      where the wing develops shocks and drag rises sharply. A subsonic polar
%      alone would under-predict cruise drag, and therefore under-size the
%      engine for the cruise and ceiling constraints. The drag-divergence
%      Mach number comes from the KORN equation (with simple sweep theory):
%          M_dd = kappa_A/cos(L) - (t/c)/cos^2(L) - CL/(10 cos^3(L))
%      kappa_A = 0.95 supercritical, 0.87 conventional; L the quarter-chord
%      sweep; t/c the mean thickness ratio. The wave drag then follows
%      LOCK'S FOURTH-POWER LAW:
%          M_crit = M_dd - (0.1/80)^(1/3)
%          CD_wave = 20 (M - M_crit)^4    for M > M_crit, else 0
%      which by construction gives dCD/dM = 0.1 at M = M_dd (the usual
%      definition of drag divergence). [Korn 1975; Lock; as presented in
%      W. H. Mason, Configuration Aerodynamics, Ch. 7 "Transonic
%      aerodynamics of airfoils and wings".]
%      Because M_dd depends on CL, the wave drag is a function of CL, so the
%      total drag is computed by CD(state, CL) rather than by a fixed polar.
%
%   3. ENGINE-OUT DRAG. The TTPA's stopped-propeller increment became a
%      WINDMILLING-ENGINE increment (dCD0_windmilling), added when a
%      condition sets engine_out_drag = true.
%
%   4. AIRFOIL TYPE (JSON aerodynamics.airfoil_type, optional):
%        "supercritical"  kappa_A = 0.95  (flat-topped, aft-loaded section)
%        "conventional"   kappa_A = 0.87  (NACA 6-series style section)
%        "custom"         kappa_A = aerodynamics.korn_technology_factor
%      Files without airfoil_type keep the old behaviour and read
%      korn_technology_factor directly. When airfoil_type is supercritical
%      or conventional, korn_technology_factor is not needed; if it is
%      present and DISAGREES with the type, the class warns and uses the
%      type's value. The airfoil type ONLY moves the drag-divergence Mach in
%      this model; its handling effects (pitching moment, trim drag,
%      off-design and inverted performance) are not modelled.
%
%   5. SWEEP-CORRECTED CLmax (JSON aerodynamics.CLmax_reference_sweep_deg,
%      optional). The three CLmax inputs are taken as the values AT that
%      quarter-chord sweep, and are scaled to the geometry's actual sweep
%      by simple sweep theory:
%          CLmax(L) = CLmax(L_ref) cos(L) / cos(L_ref)
%      [Raymer Sec. 12.4: wing CLmax ~ 0.9 clmax cos(L_qc), and the
%      high-lift increment ~ 0.9 dclmax (S_flapped/S) cos(L_hinge).] So
%      when you change geometry.sweep_quarter_chord_deg the takeoff,
%      landing and maneuver constraints see the CLmax LOSS (or gain) that
%      goes with it, instead of only the drag-divergence gain. Without the
%      key, CLmax is used as given at any sweep (old behaviour).
%
%   6. USE CD(state, CL) EVERYWHERE THE FLIGHT MACH MATTERS. drag_polar(state)
%      still returns the subsonic CD0/K1/K2 to satisfy the AerodynamicsBase
%      contract, but the constraint functions and the mission segments call
%      CD(state, CL), which adds the wave drag at state.mach.
%
%   7. TAIL INTERFERENCE (JSON aerodynamics.tail_interference_ratio,
%      optional, default 1). Scales the tails' share of CD0 only:
%          CD0 = Cfe S_wet/S_ref + (r - 1) Cfe (S_wet,ht + S_wet,vt)/S_ref
%      Cfe is an AVERAGE that already contains typical interference. A
%      V-tail has two root junctions instead of a conventional tail's
%      three, which Raymer (Sec. 12.5, component interference factors)
%      puts at Q ~ 1.03 for a V-tail against 1.04-1.05 for a conventional
%      tail, so r = 1.03/1.045 = 0.986 converts a conventional-tail Cfe to
%      a V-tail. Leave r = 1 when Cfe came from a build-up of the same
%      V-tailed configuration (e.g. VSPAERO on that model).
%
%   8. TRIM DRAG (JSON aerodynamics.trim_drag, optional, default off).
%      The tail load that balances the wing's pitching moment costs drag
%      twice: the tail's own induced drag, and the extra induced drag of a
%      wing that must carry the tail's download. Clean configuration only.
%      See trim_tail_CL and trim_drag below, and V_TAIL_MODEL.md.

    properties
        Cfe                         % <- aerodynamics.Cfe
        CD0_reference               % <- aerodynamics.CD0_reference (CHECK ONLY)
        kappa_A                     % Korn technology factor (from airfoil_type)
        airfoil_type = "custom"     % <- aerodynamics.airfoil_type (optional)

        CLmax_clean                 % SWEEP-CORRECTED values actually used
        CLmax_takeoff
        CLmax_landing
        CLmin_clean = NaN           % <- aerodynamics.CLmin_clean (optional, NEGATIVE),
                                    %    sweep-corrected; used by negative-g conditions
        CLmax_sweep_factor = 1      % cos(L)/cos(L_ref); 1 without a reference
        CLmax_reference_sweep_deg = NaN   % <- aerodynamics.CLmax_reference_sweep_deg

        dCD0_flaps_takeoff
        dCD0_flaps_landing
        dCD0_gear_down
        dCD0_windmilling            % <- aerodynamics.delta_CD0_windmilling_engine
        de_flaps_takeoff
        de_flaps_landing

        tail_interference_ratio = 1 % <- aerodynamics.tail_interference_ratio (item 7)
        trim = struct('enabled', false)   % <- aerodynamics.trim_drag (item 8)
    end

    properties (SetAccess = immutable)
        geom                        % injected TfGeom
    end

    properties (Dependent)
        AR                          % from the geometry
        CD0                         % clean subsonic CD0 = Cfe S_wet/S_ref
        e                           % Oswald efficiency (Raymer 12.48 / 12.49)
        K                           % 1/(pi AR e)
        LD_max                      % subsonic maximum L/D (no wave drag)
        CL_LDmax                    % lift coefficient of LD_max = sqrt(CD0/K)
    end

    methods

        %% Constructor
        function obj = TfAero(json_path, geom)
            arguments
                json_path {mustBeTextScalar, mustBeNonzeroLengthText}
                geom (1,1) GeometryBase
            end
            obj.geom = geom;

            J = jsondecode(fileread(json_path));
            A = J.aerodynamics;

            obj.Cfe           = A.Cfe;
            obj.CD0_reference = A.CD0_reference;
            obj.kappa_A       = TfAero.korn_factor(A);
            if isfield(A, 'airfoil_type') && ~isempty(A.airfoil_type)
                obj.airfoil_type = lower(string(A.airfoil_type));
            end

            % CLmax inputs, corrected from their reference sweep to the
            % geometry's sweep (see header, item 5).
            if isfield(A, 'CLmax_reference_sweep_deg') && ~isempty(A.CLmax_reference_sweep_deg)
                obj.CLmax_reference_sweep_deg = A.CLmax_reference_sweep_deg;
                obj.CLmax_sweep_factor = cosd(geom.sweep_qc_deg) / cosd(A.CLmax_reference_sweep_deg);
            end
            obj.CLmax_clean   = A.CLmax_clean   * obj.CLmax_sweep_factor;
            obj.CLmax_takeoff = A.CLmax_takeoff * obj.CLmax_sweep_factor;
            obj.CLmax_landing = A.CLmax_landing * obj.CLmax_sweep_factor;
            % Most negative clean lift coefficient (negative-g limit), scaled by
            % the same sweep factor. Optional: only negative-g conditions need it.
            if isfield(A, 'CLmin_clean') && ~isempty(A.CLmin_clean)
                if A.CLmin_clean >= 0
                    error('TfAero:CLminNotNegative', ...
                        'aerodynamics.CLmin_clean must be negative (got %.3f).', A.CLmin_clean);
                end
                obj.CLmin_clean = A.CLmin_clean * obj.CLmax_sweep_factor;
            end

            obj.dCD0_flaps_takeoff = A.delta_CD0_flaps_takeoff;
            obj.dCD0_flaps_landing = A.delta_CD0_flaps_landing;
            obj.dCD0_gear_down     = A.delta_CD0_gear_down;
            obj.dCD0_windmilling   = A.delta_CD0_windmilling_engine;
            obj.de_flaps_takeoff   = A.delta_e_flaps_takeoff;
            obj.de_flaps_landing   = A.delta_e_flaps_landing;

            if isfield(A, 'tail_interference_ratio') && ~isempty(A.tail_interference_ratio)
                obj.tail_interference_ratio = A.tail_interference_ratio;
            end
            if isfield(A, 'trim_drag') && ~isempty(A.trim_drag)
                T = A.trim_drag;
                d = struct('enabled', true, 'cm0_airfoil', NaN, ...
                           'cg_minus_wing_ac_over_mac', 0, 'tail_span_efficiency', 0.8);
                for f = string(fieldnames(d))'
                    if ~isfield(T, f) || isempty(T.(f)), T.(f) = d.(f); end
                end
                T.enabled = logical(T.enabled);
                if T.enabled && ~isfinite(T.cm0_airfoil)
                    error('TfAero:NoCm0', 'aerodynamics.trim_drag needs cm0_airfoil (section zero-lift moment).');
                end
                obj.trim = T;
            end
        end

        %% Dependent properties
        function val = get.AR(obj)
            val = obj.geom.AR;
        end

        function val = get.CD0(obj)
            % [Raymer Eq. 12.23] equivalent skin friction x wetted-area ratio
            val = obj.Cfe * obj.geom.S_wet_over_S_ref;
            % Tail interference correction (header item 7; 0 when r = 1)
            g   = obj.geom;
            val = val + (obj.tail_interference_ratio - 1) * obj.Cfe ...
                      * (g.S_wet_ht + g.S_wet_vt) / g.S_ref;
        end

        function val = get.e(obj)
            LE = obj.geom.sweep_LE_deg;
            if LE <= 30
                val = 1.78 * (1 - 0.045 * obj.AR^0.68) - 0.64;                    % Raymer 12.48
            else
                val = 4.61 * (1 - 0.045 * obj.AR^0.68) * cosd(LE)^0.15 - 3.1;     % Raymer 12.49
            end
            if ~(val > 0 && val <= 1)
                error('TfAero:InvalidOswaldEfficiency', ...
                    'Oswald efficiency e = %.4f is outside (0, 1].', val);
            end
        end

        function val = get.K(obj)
            val = 1 / (pi * obj.AR * obj.e);
        end

        function val = get.LD_max(obj)
            % Subsonic: 1/(2 sqrt(CD0 K)). A jet cruising above M_crit will
            % NOT reach this - use CD(state, CL) for the real cruise L/D.
            val = 1 / (2 * sqrt(obj.CD0 * obj.K));
        end

        function val = get.CL_LDmax(obj)
            val = sqrt(obj.CD0 / obj.K);
        end

        %% AerodynamicsBase contract
        function polar = drag_polar(obj, ~)
        %DRAG_POLAR  Clean SUBSONIC polar: CD = CD0 + K1 CL^2 + K2 CL.
        %   Does not include wave drag - see CD().
            polar.CD0 = obj.CD0;
            polar.K1  = obj.K;
            polar.K2  = 0;
        end

        function CLmax = get_CLmax(obj, ~, con)
        %GET_CLMAX  CLmax of the configuration named in con.config.
            switch string(con.config)
                case "clean"
                    CLmax = obj.CLmax_clean;
                case {"takeoff_flaps_gear_up", "takeoff_flaps_gear_down"}
                    CLmax = obj.CLmax_takeoff;
                case {"landing_flaps_gear_up", "landing_flaps_gear_down"}
                    CLmax = obj.CLmax_landing;
                otherwise
                    error('TfAero:UndefinedConfiguration', ...
                        'CLmax is not defined for configuration "%s".', string(con.config));
            end
        end

        %% Total drag coefficient, including compressibility
        function CD = CD(obj, state, CL, cfg)
        %CD  Total drag coefficient at the flight state [-]. VECTORIZED in CL.
        %
        %   CD = aero.CD(state, CL)        clean airplane
        %   CD = aero.CD(state, CL, cfg)   a configuration polar from
        %                                  get_config_polar
        %
        %   CD = CD0 + K1 CL^2 + K2 CL + CD_wave(M, CL)
        %
        %   CD_wave is only added for the clean configuration: flaps and gear
        %   are only ever down at low Mach, where it is zero anyway.
            if nargin < 4
                cfg = obj.drag_polar(state);
                cfg.config = "clean";
            end
            CD = cfg.CD0 + cfg.K1 .* CL.^2 + cfg.K2 .* CL;
            if cfg.config == "clean"
                CD = CD + obj.wave_drag(state.mach, CL);
                if obj.trim.enabled
                    CD = CD + obj.trim_drag(CL, cfg.K1);
                end
            end
        end

        %% Trim (header item 8)
        function Cm0 = wing_cm0(obj)
        %WING_CM0  Wing zero-lift pitching moment about its aerodynamic centre [-].
        %
        %   Cm0_w = cm0_airfoil * A cos^2(L_qc) / (A + 2 cos L_qc)
        %
        %   [Raymer Ch. 16, wing zero-lift moment from the section value; the
        %   same finite-wing / sweep factor as DATCOM.] The twist term of
        %   that relation is omitted (twist is "reporting only" in TfGeom).
            A = obj.geom.AR;  L = obj.geom.sweep_qc_deg;
            Cm0 = obj.trim.cm0_airfoil * A * cosd(L)^2 / (A + 2 * cosd(L));
        end

        function CLh = trim_tail_CL(obj, CL)
        %TRIM_TAIL_CL  Tail lift needed to trim, referred to S_ref [-]. VECTORIZED.
        %
        %   Moment balance about the CG (positive nose up, tail lift positive UP):
        %
        %       0 = Cm0_w + CL (h_cg - h_ac,w) - CL_h L_t / MAC
        %   =>  CL_h = (Cm0_w + CL dh) MAC / L_t,      dh = h_cg - h_ac,w
        %
        %   CL_h = L_tail / (q S_ref). For a V-tail L_tail is the VERTICAL
        %   resultant of the two panels' normal forces.
        %
        %   ASSUMPTIONS
        %     - Only the wing pitching moment and the wing-lift moment are
        %       balanced: fuselage and nacelle moments, thrust-line offset and
        %       the tail's own Cm0 are neglected.
        %     - L_t = the tail arm from TfGeom (wing MAC/4 to tail MAC/4) is
        %       used as the CG-to-tail distance.
        %     - dh is a fixed input (no CG travel with fuel burn).
        %     - Total lift is unchanged: the wing carries CL - CL_h.
            dh  = obj.trim.cg_minus_wing_ac_over_mac;
            CLh = (obj.wing_cm0() + CL .* dh) .* obj.geom.MAC ./ obj.geom.L_ht;
        end

        function dCD = trim_drag(obj, CL, K1)
        %TRIM_DRAG  Drag increment of trimming [-]. VECTORIZED in CL.
        %
        %   dCD = K1 [ (CL - CL_h)^2 - CL^2 ]  +  CL_h^2 S_ref / (pi e_t b_t^2)
        %         |____ wing carries CL-CL_h ___|   |___ tail induced drag ___|
        %
        %   The second term is D_i = L_h^2 / (q pi e_t b_t^2) divided by q S_ref.
        %   b_t = geom.trim_tail_span: the tail's span for a conventional tail,
        %   the HORIZONTAL PROJECTION b_V cos(G) of the developed span for a
        %   V-tail. For a V the panels' total normal force is N = L_h/cos G
        %   and each panel's induced drag follows its developed span, so
        %       D_i = N^2/(q pi e_t b_V^2) = L_h^2/(q pi e_t (b_V cos G)^2):
        %   the same result as Munk's projected-span theorem. A V-tail
        %   therefore pays MORE trim drag than a flat tail of the same
        %   developed span - the main aerodynamic penalty of the V.
        %   e_t = trim_drag.tail_span_efficiency (default 0.8); the small
        %   non-planar span-efficiency gain of a V is ignored (conservative),
        %   and the tail's profile-drag rise with lift is ignored.
            if nargin < 3, K1 = obj.K; end
            CLh = obj.trim_tail_CL(CL);
            bt  = obj.geom.trim_tail_span;
            dCD = K1 .* ((CL - CLh).^2 - CL.^2) ...
                + CLh.^2 .* obj.geom.S_ref ./ (pi * obj.trim.tail_span_efficiency * bt^2);
        end

        function CDw = wave_drag(obj, M, CL)
        %WAVE_DRAG  Korn drag divergence + Lock's fourth-power law [-].
        %   VECTORIZED in CL. Returns 0 below the critical Mach number.
            L   = obj.geom.sweep_qc_deg;
            tc  = obj.geom.tc_mean;
            Mdd = obj.kappa_A / cosd(L) - tc / cosd(L)^2 - CL ./ (10 * cosd(L)^3);
            Mcr = Mdd - (0.1/80)^(1/3);
            dM  = max(M - Mcr, 0);
            CDw = 20 * dM.^4;
        end

        function Mdd = drag_divergence_mach(obj, CL)
        %DRAG_DIVERGENCE_MACH  Korn M_dd at a lift coefficient (for reporting).
            L   = obj.geom.sweep_qc_deg;
            Mdd = obj.kappa_A / cosd(L) - obj.geom.tc_mean / cosd(L)^2 - CL ./ (10 * cosd(L)^3);
        end

        %% Configuration polar
        function cfg = get_config_polar(obj, state, con)
        %GET_CONFIG_POLAR  Polar and CLmax of the configuration of a condition.
        %
        %   cfg.CD0    clean CD0 + flap + gear (+ windmilling engine)
        %   cfg.e      clean e + flap increment
        %   cfg.K1     1/(pi AR e_cfg)
        %   cfg.K2     0
        %   cfg.CLmax  CLmax of the configuration
        %   cfg.config the configuration name
        %
        %   Unlike TtpaAero this does NOT return a climb CL with a fixed 0.2
        %   margin. FAR 25 specifies climb SPEEDS as multiples of the stall
        %   speed (V2 = 1.2 Vs, ...), so the climb CL is CLmax/k^2 and is
        %   computed in constraint_climb_far25 from con.speed_over_stall.
            polar = obj.drag_polar(state);

            switch string(con.config)
                case "clean"
                    dCD0 = 0;                                              de = 0;
                case "takeoff_flaps_gear_up"
                    dCD0 = obj.dCD0_flaps_takeoff;                         de = obj.de_flaps_takeoff;
                case "takeoff_flaps_gear_down"
                    dCD0 = obj.dCD0_flaps_takeoff + obj.dCD0_gear_down;    de = obj.de_flaps_takeoff;
                case "landing_flaps_gear_up"
                    dCD0 = obj.dCD0_flaps_landing;                         de = obj.de_flaps_landing;
                case "landing_flaps_gear_down"
                    dCD0 = obj.dCD0_flaps_landing + obj.dCD0_gear_down;    de = obj.de_flaps_landing;
                otherwise
                    error('TfAero:UndefinedConfiguration', ...
                        'Drag polar is not defined for configuration "%s".', string(con.config));
            end

            % Failed engine: windmilling fan and inlet spillage drag
            if isfield(con, 'engine_out_drag') && con.engine_out_drag
                dCD0 = dCD0 + obj.dCD0_windmilling;
            end

            cfg.config = string(con.config);
            cfg.CD0    = polar.CD0 + dCD0;
            cfg.e      = obj.e + de;
            cfg.K1     = 1 / (pi * obj.AR * cfg.e);
            cfg.K2     = polar.K2;
            cfg.CLmax  = obj.get_CLmax(state, con);
        end
    end

    methods (Static)
        function kappa = korn_factor(A)
        %KORN_FACTOR  Korn technology factor from the aerodynamics block.
        %   airfoil_type "supercritical" -> 0.95, "conventional" -> 0.87,
        %   "custom" or absent -> A.korn_technology_factor.
        %   [Korn 1975; Mason, Configuration Aerodynamics, Ch. 7]
            table = struct('supercritical', 0.95, 'conventional', 0.87);
            has_k = isfield(A, 'korn_technology_factor') && ~isempty(A.korn_technology_factor);
            if ~isfield(A, 'airfoil_type') || isempty(A.airfoil_type) ...
                    || strcmpi(string(A.airfoil_type), "custom")
                if ~has_k
                    error('TfAero:NoKornFactor', ['Set aerodynamics.airfoil_type to ', ...
                        '"supercritical" or "conventional", or give korn_technology_factor.']);
                end
                kappa = A.korn_technology_factor;
                return
            end
            type = lower(string(A.airfoil_type));
            if ~isfield(table, type)
                error('TfAero:UndefinedAirfoil', ['aerodynamics.airfoil_type must be ', ...
                    '"supercritical", "conventional" or "custom", not "%s".'], type);
            end
            kappa = table.(type);
            if has_k && abs(A.korn_technology_factor - kappa) > 1e-12
                warning('TfAero:KornFactorIgnored', ['airfoil_type "%s" sets kappa_A = %.2f; ', ...
                    'korn_technology_factor = %.3f is ignored (use airfoil_type "custom" to use it).'], ...
                    type, kappa, A.korn_technology_factor);
            end
        end
    end
end
