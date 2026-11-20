classdef TfWeights < WeightsBase
%TFWEIGHTS  Component weight build-up for a twin-turbofan airplane.
%
%   wts = TfWeights(json_path, geom, prop)
%
%   Same structure as TtpaWeights - Raymer Table 15.2's seven-row build-up -
%   with the TRANSPORT/BOMBER column in place of the general-aviation one,
%   because a pressurised, high-altitude, 0.75-Mach airplane is built far
%   heavier per square foot than a light piston twin:
%
%                        GA (TTPA)          Transport (this class)
%       wing             2.5 psf exposed    10   psf exposed
%       horizontal tail  2.0 psf            5.5  psf
%       vertical tail    2.0 psf            5.5  psf
%       fuselage         1.4 psf wetted     5.0  psf wetted
%       landing gear     0.057 W_TO         0.043 W_TO
%       installed engine 1.4 x bare         1.3  x bare
%       all else empty   0.10 W_TO          0.17 W_TO
%
%   All seven numbers are READ from the weights block, so you can change the
%   column without editing this file.
%
%   OEW(W_TO) evaluates every W_TO-dependent row at the PASSED weight, never
%   at obj.W_TO, for the reason TtpaWeights gives: the sizing loop calls it
%   at a trial weight that is not yet the object's weight.
%
%   The statistical cross-check is now Raymer Table 3.1 (jet transport):
%       W_e/W_0 = 1.02 W_0^-0.06   ->   OEW_statistical = 1.02 W_0^0.94
%   KEPT ONLY AS A CHECK; the loop never calls it.

    properties
        W_TO                 = NaN   % lbf, candidate gross takeoff weight
        W_energy             = NaN   % lbf, mission fuel
        W_payload_expendable = 0     % lbf
        W_payload_fixed      = NaN   % lbf, crew + passengers + baggage
    end

    properties (SetAccess = immutable)
        geom                         % injected TfGeom
        prop                         % injected TfProp

        rho_wing                     % psf of EXPOSED wing planform
        rho_ht                       % psf of horizontal-tail area
        rho_vt                       % psf of vertical-tail area
        rho_fuselage                 % psf of fuselage WETTED area
        f_landing_gear               % x W_TO
        f_installed_engine           % x bare engine weight of all engines
        f_all_else_empty             % x W_TO

        k_empty_statistical          % Raymer Table 3.1 coefficient
        n_empty_statistical          % Raymer Table 3.1 exponent

        rho_fuel                     % lb/ft^3, post-sizing check only

        % --- wing weight method (optional; default keeps the unit weight) ---
        wing_method = "unit"         % <- weights.wing_weight_method: "unit" | "raymer_fighter"
        wr = struct()                % <- weights.wing_raymer (see wing_weight_raymer)

        % --- V-tail weight (only when geometry.tail_type = "v_tail") ---
        vtw = struct('method', "none")   % <- weights.v_tail (see v_tail_weight)

        % --- twin-fin weight (only when geometry.tail_type = "twin_vertical") ---
        tfw = struct('method', "none")   % <- weights.twin_vertical (see fin_weight)
    end

    methods
        %% Constructor
        function obj = TfWeights(json_path, geom, prop)
            arguments
                json_path {mustBeTextScalar, mustBeNonzeroLengthText}
                geom (1,1) GeometryBase
                prop (1,1) ThrustPropulsionBase
            end
            obj.geom = geom;
            obj.prop = prop;

            J = jsondecode(fileread(json_path));
            W = J.weights;

            obj.W_payload_fixed      = W.W_payload_fixed;
            obj.W_payload_expendable = W.W_payload_expendable;

            obj.rho_wing           = W.unit_weight_wing_psf;
            obj.rho_ht             = W.unit_weight_horizontal_tail_psf;
            obj.rho_vt             = W.unit_weight_vertical_tail_psf;
            obj.rho_fuselage       = W.unit_weight_fuselage_psf;
            obj.f_landing_gear     = W.landing_gear_fraction;
            obj.f_installed_engine = W.installed_engine_factor;
            obj.f_all_else_empty   = W.all_else_empty_fraction;

            obj.k_empty_statistical = W.statistical_empty_coefficient;
            obj.n_empty_statistical = W.statistical_empty_exponent;

            obj.rho_fuel = W.fuel_density_lb_per_ft3;

            % Optional wing-weight method. "unit" (default): rho_wing x exposed
            % area - no effect of AR, sweep, thickness or load factor.
            % "raymer_fighter": Raymer's fighter/attack wing equation, which
            % does respond to them (see wing_weight_raymer).
            if isfield(W, 'wing_weight_method') && ~isempty(W.wing_weight_method)
                obj.wing_method = lower(string(W.wing_weight_method));
            end
            switch obj.wing_method
                case "unit"
                case "raymer_fighter"
                    if ~isfield(W, 'wing_raymer')
                        error('TfWeights:NoRaymerBlock', ...
                            'wing_weight_method = "raymer_fighter" needs a weights.wing_raymer block.');
                    end
                    R = W.wing_raymer;
                    d = struct('limit_load_factor', 7, 'ultimate_factor', 1.5, 'K_dw', 1, 'K_vs', 1, ...
                               'control_surface_fraction', 0.12, 'calibration_factor', 1);
                    for f = string(fieldnames(d))'
                        if ~isfield(R, f) || isempty(R.(f)), R.(f) = d.(f); end
                    end
                    % limit_load_factor = "auto": the structure is designed to
                    % the LARGEST positive load factor the requirements ask for
                    % (sustained-turn 'flight' conditions and instantaneous
                    % turns). The PRM2 RFP ties the limit load factor to the
                    % sustained-g requirement (RFP-3-P02-R03.1.1), so a
                    % threshold file gets 4.0 g, an objective file 6.0 g and a
                    % file with a +7 g instantaneous requirement 7.0 g - the
                    % structure always matches the file's own requirements.
                    if ischar(R.limit_load_factor) || isstring(R.limit_load_factor)
                        if ~strcmpi(string(R.limit_load_factor), "auto")
                            error('TfWeights:BadLimitLoadFactor', ...
                                'wing_raymer.limit_load_factor must be a number or "auto".');
                        end
                        [R.limit_load_factor, R.limit_source] = TfWeights.requirement_load_factor(J);
                    else
                        R.limit_source = "given in weights.wing_raymer";
                    end
                    obj.wr = R;
                otherwise
                    error('TfWeights:UndefinedWingMethod', ...
                        'weights.wing_weight_method must be "unit" or "raymer_fighter", not "%s".', obj.wing_method);
            end

            % V-tail weight block (optional; see v_tail_weight). Without it a
            % V-tail is weighed with the unit-weight blend, which equals the
            % old split-tail answer exactly.
            if isprop(geom, 'tail_type') && geom.tail_type == "v_tail"
                % unit_weight_psf NaN = the blend (computed when weighed,
                % since a "projected_area" V's dihedral follows S_ref)
                d = struct('method', "unit", 'unit_weight_psf', NaN, ...
                           'penalty_factor', 1, 'calibration_factor', 1, ...
                           'limit_load_factor', "wing", 'ultimate_factor', 1.5);
                T = struct();
                if isfield(W, 'v_tail') && ~isempty(W.v_tail), T = W.v_tail; end
                for f = string(fieldnames(d))'
                    if ~isfield(T, f) || isempty(T.(f)), T.(f) = d.(f); end
                end
                T.method = lower(string(T.method));
                if ~ismember(T.method, ["unit", "raymer_fighter"])
                    error('TfWeights:UndefinedVTailMethod', ...
                        'weights.v_tail.method must be "unit" or "raymer_fighter", not "%s".', T.method);
                end
                % Load factor: "wing" = the wing's limit (wing_raymer), "auto" =
                % the largest in the requirements, or a number.
                n = T.limit_load_factor;
                if ischar(n) || isstring(n)
                    switch lower(string(n))
                        case "wing"
                            if obj.wing_method == "raymer_fighter"
                                n = obj.wr.limit_load_factor;
                            else
                                n = TfWeights.requirement_load_factor(J);
                            end
                        case "auto"
                            n = TfWeights.requirement_load_factor(J);
                        otherwise
                            error('TfWeights:BadVTailLoadFactor', ...
                                'weights.v_tail.limit_load_factor must be a number, "wing" or "auto".');
                    end
                end
                T.limit_load_factor = n;
                obj.vtw = T;
            end

            % Twin-fin weight block (optional; see fin_weight). Without it the
            % fins keep the unit weight on their TRUE area.
            if isprop(geom, 'tail_type') && geom.tail_type == "twin_vertical"
                d = struct('method', "unit", 'fin_area_basis', "per_fin", ...
                           'penalty_factor', 1, 'calibration_factor', 1, ...
                           'limit_load_factor', "wing", 'ultimate_factor', 1.5, ...
                           'max_mach', 0.8, 'rudder_area_fraction', 0.25, ...
                           'K_rht', 1.0, 'Ht_over_Hv', 0);
                T = struct();
                if isfield(W, 'twin_vertical') && ~isempty(W.twin_vertical), T = W.twin_vertical; end
                for f = string(fieldnames(d))'
                    if ~isfield(T, f) || isempty(T.(f)), T.(f) = d.(f); end
                end
                T.method = lower(string(T.method));
                T.fin_area_basis = lower(string(T.fin_area_basis));
                if ~ismember(T.method, ["unit", "raymer_fighter"])
                    error('TfWeights:UndefinedFinMethod', ...
                        'weights.twin_vertical.method must be "unit" or "raymer_fighter", not "%s".', T.method);
                end
                if ~ismember(T.fin_area_basis, ["per_fin", "total"])
                    error('TfWeights:BadFinBasis', ...
                        'weights.twin_vertical.fin_area_basis must be "per_fin" or "total".');
                end
                T.limit_load_factor = obj.resolve_load_factor_(T.limit_load_factor, J);
                obj.tfw = T;
            end
        end

        %% Twin-fin weight
        function [W, raw_each] = fin_weight(obj, W_dg)
        %FIN_WEIGHT  Weight of ALL vertical fins of a twin-fin tail [lbf].
        %
        %   method "unit":   W = K_F rho_vt S_vt        (S_vt = TRUE fin area)
        %
        %   method "raymer_fighter":
        %     [Raymer Eq. 15.3, fighter/attack VERTICAL tail, for one surface]
        %
        %     W_1(S) = 0.452 K_rht (1 + H_t/H_v)^0.5 (W_dg N_z)^0.488 S^0.718
        %              M^0.341 L_t^-1.0 (1 + S_r/S)^0.348 A^0.223 (1 + lambda)^0.25
        %              (cos L_vt)^-0.323
        %
        %     fin_area_basis "per_fin" (default):  W = K_F k n W_1(S_vt/n)
        %     fin_area_basis "total":              W = K_F k   W_1(S_vt)
        %
        %     S        fin TRUE area [ft^2]            A      ONE fin's aspect ratio
        %     L_t      tail arm [ft]                   lambda fin taper
        %     L_vt     fin quarter-chord sweep         M      design max Mach (max_mach)
        %     S_r/S    rudder_area_fraction            K_rht  1.047 rolling tail, else 1
        %     H_t/H_v  0 conventional, 1 T-tail        N_z    ULTIMATE load factor
        %
        %   "per_fin" treats each fin as its own surface, which is how the
        %   equation is applied to a multi-fin layout here. Because S enters
        %   as S^0.718, two fins of half the area weigh 2^(1-0.718) = 1.22 x
        %   one fin of the full area - the structural price of splitting the
        %   fin (two roots, two attachments, two sets of actuators). "total"
        %   gives the single-fin answer for comparison; the truth for a given
        %   design is in between and depends on how much structure the fins
        %   share.
        %
        %   CALIBRATION (k = calibration_factor). Anchored at the T-45C like
        %   the wing and the V-tail: W_1 for the T-45C model's single fin at
        %   its weight and load factor equals 5.3 psf x its area. The
        %   empty-weight calibration (all_else_empty_fraction) used that unit
        %   weight, so it still holds. M, S_r/S, K_rht and H_t/H_v are taken
        %   the same at the anchor and on the concept, so they cancel unless
        %   you change them for the concept.
        %
        %   K_F = penalty_factor (default 1) for anything else (e.g. a boom
        %   or fin-fuselage fairing structure).
        %
        %   raw_each is W_1 of one fin (or of the total, basis "total"),
        %   before k and K_F.
            T = obj.tfw;  g = obj.geom;
            switch T.method
                case "unit"
                    raw_each = obj.rho_vt * g.S_fin;
                    W = T.penalty_factor * obj.rho_vt * g.S_vt;
                case "raymer_fighter"
                    if T.fin_area_basis == "per_fin"
                        S = g.S_fin;  n = g.n_fins;
                    else
                        S = g.S_vt;   n = 1;
                    end
                    Nz = T.limit_load_factor * T.ultimate_factor;
                    raw_each = TfWeights.raymer_fighter_vt(W_dg, Nz, S, T.max_mach, g.L_vt, ...
                        T.rudder_area_fraction, g.AR_vt, g.taper_vt, g.sweep_qc_vt_deg, T.K_rht, T.Ht_over_Hv);
                    W = T.penalty_factor * T.calibration_factor * n * raw_each;
            end
        end

        function n = resolve_load_factor_(obj, n, J)
        %RESOLVE_LOAD_FACTOR_  "wing" -> the wing's limit, "auto" -> requirements, or a number.
            if ischar(n) || isstring(n)
                switch lower(string(n))
                    case "wing"
                        if obj.wing_method == "raymer_fighter"
                            n = obj.wr.limit_load_factor;
                        else
                            n = TfWeights.requirement_load_factor(J);
                        end
                    case "auto"
                        n = TfWeights.requirement_load_factor(J);
                    otherwise
                        error('TfWeights:BadLoadFactor', ...
                            'limit_load_factor must be a number, "wing" or "auto".');
                end
            end
        end

        %% V-tail weight
        function [W, raw] = v_tail_weight(obj, W_dg)
        %V_TAIL_WEIGHT  Weight of BOTH V-tail panels [lbf].
        %
        %   method "unit":
        %       W = K_V rho_V S_V
        %       rho_V default = rho_ht cos^2 G + rho_vt sin^2 G, the unit
        %       weights of the two surfaces the V replaces, weighted by the
        %       share of its area that does each job. With K_V = 1 this is
        %       exactly the old "split into two equivalent tails" answer.
        %
        %   method "raymer_fighter":
        %       W = K_V k_cal 3.316 (1 + F_w/b_V)^-2.0 (W_dg N_z/1000)^0.260 S_V^0.806
        %       [Raymer Eq. 15.2, fighter/attack HORIZONTAL tail]
        %
        %       S_V    total panel area (both panels)            [ft^2]
        %       b_V    developed span, tip to tip along panels   [ft]
        %       F_w    fuselage width at the tail root           [ft]
        %       N_z    ULTIMATE load factor = limit x ultimate_factor
        %       W_dg   design gross weight (the W_TO passed in)
        %
        %   WHY THE HORIZONTAL-TAIL EQUATION. Raymer gives no V-tail equation.
        %   A V-tail's panels are all-moving or ruddervator-equipped lifting
        %   surfaces whose root bending is driven mainly by the pitch
        %   (maneuver) load, which the horizontal-tail equation scales with
        %   W_dg N_z; the fighter vertical-tail equation has no N_z term.
        %   The whole V area is used, so the economy of scale (S^0.806) of
        %   ONE structure instead of two is kept. F_w/b_V uses the DEVELOPED
        %   span because the carry-through structure follows the panels.
        %
        %   CALIBRATION (k_cal = calibration_factor). The PRM2 files anchor
        %   it at the T-45C, like the wing: a V with the T-45C's total tail
        %   area at the T-45C's weight and load factor weighs what that
        %   airplane's two unit-weight tails weigh (4.0 and 5.3 psf). The
        %   empty-weight calibration (all_else_empty_fraction) was made
        %   with those unit-weight tails, so it still holds; only the
        %   equation's TRENDS (size, W_dg, N_z) change the answer.
        %
        %   K_V = penalty_factor (both methods, default 1): a structural
        %   penalty for the V's root fitting, which carries combined pitch,
        %   yaw and torsion loads, and the ruddervator mixing hardware. No
        %   statistical value is published that this code can cite; leave
        %   it at 1 or set your own judgement and say so.
        %
        %   raw is the value before K_V and k_cal.
            T = obj.vtw;  g = obj.geom;
            S = g.S_vtail;
            switch T.method
                case "unit"
                    rho = T.unit_weight_psf;
                    if ~isfinite(rho)
                        Gam = g.vtail_dihedral_deg;
                        rho = obj.rho_ht * cosd(Gam)^2 + obj.rho_vt * sind(Gam)^2;
                    end
                    raw = rho * S;
                    W   = T.penalty_factor * raw;
                case "raymer_fighter"
                    Nz  = T.limit_load_factor * T.ultimate_factor;
                    raw = 3.316 * (1 + g.vtail.fuselage_width_at_tail_ft / g.b_vtail)^(-2.0) ...
                        * (W_dg * Nz / 1000)^0.260 * S^0.806;
                    W   = T.penalty_factor * T.calibration_factor * raw;
            end
        end

        %% Raymer fighter/attack wing weight
        function [W, raw] = wing_weight_raymer(obj, W_dg)
        %WING_WEIGHT_RAYMER  Wing weight that responds to AR, sweep, t/c and load factor [lbf].
        %
        %   [W, raw] = wing_weight_raymer(obj, W_dg)
        %
        %   [Raymer, Aircraft Design: A Conceptual Approach, Eq. 15.1,
        %   fighter/attack wing]
        %
        %     W = 0.0103 K_dw K_vs (W_dg N_z)^0.5 S_w^0.622 A^0.785
        %                (t/c)_root^-0.4 (1 + lambda)^0.05 (cos L_qc)^-1 S_csw^0.04
        %
        %     W_dg    design gross weight [lb] - the takeoff weight passed in
        %     N_z     ULTIMATE load factor = limit_load_factor x ultimate_factor
        %             (limit_load_factor "auto" = the largest positive n in
        %             the file's requirements, see requirement_load_factor)
        %     S_w     trapezoidal (reference) wing area [ft^2]
        %     A       aspect ratio;   lambda  taper;   L_qc quarter-chord sweep
        %     S_csw   wing-mounted control-surface area [ft^2]
        %             = control_surface_fraction x S_w
        %     K_dw    0.768 for a delta wing, else 1
        %     K_vs    1.19 for variable sweep, else 1
        %
        %   WHY. The unit-weight method (rho_wing x exposed area) makes a
        %   higher-aspect-ratio, more swept or thinner wing FREE structurally,
        %   so any trade of those parameters favours them without limit.
        %   Raymer's equation carries the real penalties: A^0.785 (longer span,
        %   bigger root bending moment), (t/c)^-0.4 (shallower spar), 1/cos L
        %   (longer structural span) and sqrt(N_z) (stronger structure).
        %
        %   CALIBRATION. The result is multiplied by calibration_factor
        %   (weights.wing_raymer). The PRM2 files set it so the equation gives
        %   the SAME wing weight as the unit method on the airplane the rest of
        %   the weight model was calibrated on (the L-39C). The calibrated
        %   empty weight therefore still holds there, and only the TRENDS of
        %   the equation move the answer for other wings.
        %
        %   raw is the uncalibrated equation value.
            R  = obj.wr;
            g  = obj.geom;
            Nz = R.limit_load_factor * R.ultimate_factor;
            S  = g.S_ref;
            raw = 0.0103 * R.K_dw * R.K_vs * (W_dg * Nz)^0.5 * S^0.622 * g.AR^0.785 ...
                * g.tc_root^(-0.4) * (1 + g.taper)^0.05 / cosd(g.sweep_qc_deg) ...
                * (R.control_surface_fraction * S)^0.04;
            W = R.calibration_factor * raw;
        end

        %% Operating empty weight (the WeightsBase contract)
        function oew = OEW(obj, W_TO)
            arguments
                obj
                W_TO (1,1) double {mustBePositive}
            end
            bd  = obj.OEW_breakdown(W_TO);
            oew = bd.total;
        end

        %% The seven rows
        function bd = OEW_breakdown(obj, W_TO)
        %OEW_BREAKDOWN  The seven rows of Raymer Table 15.2 [lbf].
        %   Four follow the geometry (wing, both tails, fuselage), one follows
        %   the ENGINE (and therefore T_SL), two follow W_TO.
            arguments
                obj
                W_TO (1,1) double {mustBePositive}
            end
            [W_bare_total, W_bare_each] = obj.prop.engine_weight();   % Raymer Eq. 10.1
            bd.engine_bare_each  = W_bare_each;
            bd.engine_bare_total = W_bare_total;

            if obj.wing_method == "raymer_fighter"
                bd.wing         = obj.wing_weight_raymer(W_TO);          % PASSED W_TO
            else
                bd.wing         = obj.rho_wing     * obj.geom.S_exposed_wing;
            end
            if obj.vtw.method ~= "none"
                % V-tail: one structure. Its weight is reported in bd.v_tail
                % and, so existing tables still add up, split over the two
                % tail rows in the pitch/yaw shares of its area.
                bd.v_tail           = obj.v_tail_weight(W_TO);            % PASSED W_TO
                Gam = obj.geom.vtail_dihedral_deg;
                bd.horizontal_tail  = bd.v_tail * cosd(Gam)^2;
                bd.vertical_tail    = bd.v_tail * sind(Gam)^2;
            elseif obj.tfw.method ~= "none"
                % Twin fins: horizontal tail by unit weight as before, the
                % fins by fin_weight on their TRUE area.
                bd.horizontal_tail  = obj.rho_ht       * obj.geom.S_ht;
                bd.vertical_tail    = obj.fin_weight(W_TO);               % PASSED W_TO
            else
                bd.horizontal_tail  = obj.rho_ht       * obj.geom.S_ht;
                bd.vertical_tail    = obj.rho_vt       * obj.geom.S_vt;
            end
            bd.fuselage         = obj.rho_fuselage * obj.geom.S_wet_fuselage;

            bd.landing_gear     = obj.f_landing_gear     * W_TO;          % PASSED W_TO
            bd.installed_engine = obj.f_installed_engine * W_bare_total;
            bd.all_else_empty   = obj.f_all_else_empty   * W_TO;          % PASSED W_TO

            bd.total = bd.wing + bd.horizontal_tail + bd.vertical_tail + bd.fuselage ...
                     + bd.landing_gear + bd.installed_engine + bd.all_else_empty;
        end

        %% Cross-check
        function oew = OEW_statistical(obj, W_TO)
        %OEW_STATISTICAL  Raymer Table 3.1 jet-transport trend [lbf].
        %   W_e = A W_0^(1+C), from W_e/W_0 = A W_0^C.
            oew = obj.k_empty_statistical * W_TO^(1 + obj.n_empty_statistical);
        end

        %% Payload
        function W = payload(obj)
            W = obj.W_payload_fixed + obj.W_payload_expendable;
        end
    end

    methods (Static)
        function W = raymer_fighter_vt(W_dg, Nz, S, M, L_t, Sr_frac, A, lambda, sweep_deg, K_rht, Ht_Hv)
        %RAYMER_FIGHTER_VT  Raymer Eq. 15.3, fighter/attack vertical tail, ONE surface [lb].
            W = 0.452 * K_rht * (1 + Ht_Hv)^0.5 * (W_dg * Nz)^0.488 * S^0.718 * M^0.341 ...
              * L_t^(-1.0) * (1 + Sr_frac)^0.348 * A^0.223 * (1 + lambda)^0.25 ...
              * cosd(sweep_deg)^(-0.323);
        end

        function [n_lim, src] = requirement_load_factor(J)
        %REQUIREMENT_LOAD_FACTOR  Largest positive load factor in the constraint set.
        %   Looks at every 'flight' condition (sustained turns, n) and every
        %   'instantaneous_turn' condition with n > 0. Returns 1 if there is
        %   none, and the name of the condition that sets it.
            n_lim = 1;  src = "no maneuver requirement (1 g)";
            C = json_as_struct_array(J.constraints.conditions);
            for k = 1:numel(C)
                c = C(k);
                if ~isfield(c, 'type') || ~isfield(c, 'n') || isempty(c.n), continue, end
                if ismember(string(c.type), ["flight", "instantaneous_turn"]) && c.n > n_lim
                    n_lim = c.n;
                    src = sprintf('"%s" (n = %.2f)', string(c.name), c.n);
                end
            end
        end
    end
end
