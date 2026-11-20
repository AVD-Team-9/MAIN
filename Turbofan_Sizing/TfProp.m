classdef TfProp < ThrustPropulsionBase
%TFPROP  Preliminary high-bypass turbofan propulsion model.
%
%   prop = TfProp(json_path)  reads the propulsion block of the requirements
%   file. Then set prop.T_SL (the sizing loop does this every pass).
%
%   This class REPLACES TtpaProp. Side by side:
%
%       TtpaProp (piston + propeller)        TfProp (turbofan)
%       -----------------------------        ------------------------------
%       P_SL       shaft power, hp           T_SL       thrust, lbf
%       power_lapse = 1.132 sigma - 0.132    thrust_lapse = Mattingly HBTF
%                     (density only)                       (theta_0, delta_0, M)
%       power_ratio                          thrust_ratio
%       BSFC  lbm/(hp hr), constant          tsfc  1/hr, varies with M and theta
%       prop_eff  eta_p                      (none - a jet has no propeller)
%       engine_weight  Raymer Table 10.4     engine_weight  Raymer Eq. 10.1
%       engine_length  Raymer Table 10.4     engine_length  Raymer Eq. 10.2
%       (diameter fixed in geometry)         engine_diameter Raymer Eq. 10.3
%
%   TWO ENGINE TYPES, picked by lapse_model and tsfc_model in the JSON:
%       "mattingly_hbtf"   high-bypass turbofan (BPR ~4 and up)
%       "mattingly_lbtf"   low-bypass mixed-flow turbofan, DRY (no
%                          afterburner) - for subsonic airplanes with
%                          BPR below about 1.5
%   Set BOTH keys to the same type, and set bypass_ratio to match, because
%   the engine weight and diameter regressions read it. tsfc_model can also
%   be "constant".
%
%   WHAT T_SL MEANS. T_SL is the INSTALLED, SEA-LEVEL-STATIC (M = 0), TAKEOFF-
%   RATED thrust of ALL engines together. Every constraint and every mission
%   segment is written as a fraction of it:
%
%       thrust available = T_SL * thrust_lapse(state, rating) * (OEI) * throttle
%
%   so the constraint diagram's vertical axis, (T/W)_TO, is T_SL / W_TO.
%   The engine-size regressions use the thrust of ONE engine, T_SL/n_engines.
%   uninstalled_thrust() converts to the catalogue (uninstalled) figure.
%
%   RUBBER OR FIXED ENGINE (JSON propulsion.engine_sizing, optional):
%       "rubber" (default)  the sizing loop scales T_SL to the least-thrust
%                           corner every pass, and the engine weight, length
%                           and diameter follow it through Raymer Eqs. 10.1-3.
%       "fixed"             a REAL candidate engine: propulsion.fixed_engine
%                           gives its catalogue (uninstalled) thrust, dry
%                           weight, length and diameter. T_SL is held at
%                           n x thrust x (1 - installation loss) and the
%                           engine size comes from the data, not the
%                           regression. The sizing loop then sizes only the
%                           WEIGHT and the WING around that engine (see
%                           sizing_pass and fixed_engine_wing_loading).
%
%   It is a HANDLE class, like every discipline here, so when the sizing loop
%   writes prop.T_SL the geometry (nacelles) and the weights (engines) that
%   hold a reference to this object see the new engine on their next read.

    properties
        engine_type = "High-bypass turbofan"

        T_SL = NaN
        % Installed sea-level-static takeoff thrust of ALL engines [lbf].
        % The sizing-loop STATE that replaces P_SL. NaN until set.

        n_engines                    % <- J.propulsion.n_engines
        bypass_ratio                 % <- J.propulsion.bypass_ratio

        lapse_model                  % <- J.propulsion.lapse_model
        throttle_ratio               % <- J.propulsion.throttle_ratio  (TR)
        tsfc_model                   % <- J.propulsion.tsfc_model
        tsfc_constant                % <- J.propulsion.tsfc_constant_per_hr
        tsfc_scale = 1               % <- J.propulsion.tsfc_scale (optional, default 1)
                                     %    multiplies the Mattingly TSFC; use it to
                                     %    calibrate the correlation to a real engine
        T_TO_over_T_max_continuous   % <- J.propulsion.T_TO_over_T_max_continuous
        installation_loss            % <- J.propulsion.installation_loss_fraction
        design_mach                  % <- J.propulsion.design_mach

        % --- Raymer Eqs. 10.1-10.3 rubber-engine regression, PER ENGINE ---
        kW                           % weight:   W = kW * T^nW * exp(bW * BPR)
        nW
        bW
        kL                           % length:   L = kL * T^nL * M^mL
        nL
        mL
        kD                           % diameter: D = kD * T^nD * exp(bD * BPR)
        nD
        bD
        T_range                      % <- thrust_range_of_validity_lbf

        % --- fixed (real) engine, used when engine_sizing = "fixed" ---
        engine_sizing = "rubber"     % <- propulsion.engine_sizing
        fixed_engine  = struct()     % <- propulsion.fixed_engine: name, thrust_lbf,
                                     %    dry_weight_lbf, length_ft, diameter_ft
    end

    properties (Dependent)
        T_engine
        % Installed sea-level-static takeoff thrust of ONE engine [lbf].
    end

    methods

        %% Constructor
        function obj = TfProp(json_path)
            arguments
                json_path {mustBeTextScalar, mustBeNonzeroLengthText}
            end

            J = jsondecode(fileread(json_path));
            P = J.propulsion;

            obj.engine_type    = string(P.engine_type);
            obj.n_engines      = P.n_engines;
            obj.bypass_ratio   = P.bypass_ratio;

            obj.lapse_model    = string(P.lapse_model);
            obj.throttle_ratio = P.throttle_ratio;
            obj.tsfc_model     = string(P.tsfc_model);
            obj.tsfc_constant  = P.tsfc_constant_per_hr;
            if isfield(P, 'tsfc_scale') && ~isempty(P.tsfc_scale)
                obj.tsfc_scale = P.tsfc_scale;      % optional calibration factor
            end
            obj.T_TO_over_T_max_continuous = P.T_TO_over_T_max_continuous;
            obj.installation_loss          = P.installation_loss_fraction;
            obj.design_mach                = P.design_mach;

            obj.kW = P.engine_weight_coefficient;
            obj.nW = P.engine_weight_exponent;
            obj.bW = P.engine_weight_bpr_coefficient;
            obj.kL = P.engine_length_coefficient;
            obj.nL = P.engine_length_exponent;
            obj.mL = P.engine_length_mach_exponent;
            obj.kD = P.engine_diameter_coefficient;
            obj.nD = P.engine_diameter_exponent;
            obj.bD = P.engine_diameter_bpr_coefficient;
            obj.T_range = P.thrust_range_of_validity_lbf;

            if isfield(P, 'engine_sizing') && ~isempty(P.engine_sizing)
                obj.engine_sizing = lower(string(P.engine_sizing));
            end
            switch obj.engine_sizing
                case "rubber"
                case "fixed"
                    if ~isfield(P, 'fixed_engine')
                        error('TfProp:NoFixedEngine', ...
                            'engine_sizing = "fixed" needs a propulsion.fixed_engine block.');
                    end
                    F = P.fixed_engine;
                    need = {'thrust_lbf', 'dry_weight_lbf', 'length_ft', 'diameter_ft'};
                    for i = 1:numel(need)
                        if ~isfield(F, need{i}) || isempty(F.(need{i}))
                            error('TfProp:FixedEngineField', ...
                                'propulsion.fixed_engine needs "%s".', need{i});
                        end
                    end
                    if ~isfield(F, 'name'), F.name = "fixed engine"; end
                    obj.fixed_engine = F;
                    obj.T_SL = obj.T_SL_fixed();     % the engine IS the engine
                otherwise
                    error('TfProp:UndefinedEngineSizing', ...
                        'propulsion.engine_sizing must be "rubber" or "fixed", not "%s".', obj.engine_sizing);
            end
        end


        %% Fixed engine
        function tf = is_fixed(obj)
        %IS_FIXED  true when the airplane is sized around a real, fixed engine.
            tf = obj.engine_sizing == "fixed";
        end

        function T = T_SL_fixed(obj)
        %T_SL_FIXED  Installed SLS thrust of ALL the fixed engines [lbf]:
        %   n x catalogue thrust x (1 - installation loss).
            if ~obj.is_fixed()
                error('TfProp:NotFixed', 'The engine is a rubber engine.');
            end
            T = obj.n_engines * obj.fixed_engine.thrust_lbf * (1 - obj.installation_loss);
        end


        %% Thrust per engine
        function val = get.T_engine(obj)
            val = obj.T_SL / obj.n_engines;
        end


        %% Thrust lapse
        function alpha = thrust_lapse(obj, state, rating)
        %THRUST_LAPSE  Thrust at the state and rating / T_SL [-].
        %
        %   alpha = thrust_lapse(obj, state, rating)
        %     state   struct from get_state(alt, mach); needs theta_0,
        %             delta_0 and mach
        %     rating  "takeoff" (full rating) or "max_continuous"
        %
        %   "mattingly_hbtf"  [Mattingly, Heiser & Pratt, Aircraft Engine
        %   Design, 2nd ed., Eq. 2.54a, high-bypass turbofan, max power]
        %
        %     theta_0 <= TR:  alpha = delta_0 (1 - 0.49 sqrt(M0))
        %     theta_0 >  TR:  alpha = delta_0 (1 - 0.49 sqrt(M0)
        %                                    - 3 (theta_0 - TR)/(1.5 + M0))
        %
        %   WHY IT LOOKS LIKE THIS. delta_0 is the total pressure at the
        %   engine face over sea-level pressure: thrust scales with the mass
        %   flow the engine can swallow, which scales with it. The
        %   1 - 0.49 sqrt(M) term is the fan's loss of net thrust with flight
        %   speed (ram drag grows faster than gross thrust for a high-bypass
        %   fan). Above the throttle ratio TR the engine hits its turbine
        %   temperature limit and must throttle back - that is the second
        %   branch. At 41,000 ft and M 0.75 this gives alpha ~ 0.15: the
        %   engine makes only about 15 % of its static thrust in cruise,
        %   which is why cruise and ceiling constraints so often size a
        %   business jet's engine.
        %
        %   At sea level static (theta_0 = delta_0 = 1, M = 0) alpha = 1 by
        %   construction, so T_SL really is the static thrust.
        %
        %   The rating factor multiplies on top, exactly as in TtpaProp:
        %   "max_continuous" is the takeoff thrust / T_TO_over_T_max_continuous.

            M = state.mach;

            switch obj.lapse_model
                case "mattingly_hbtf"
                    % ---- HIGH-bypass turbofan (BPR ~4 and up) ---------------
                    if state.theta_0 <= obj.throttle_ratio
                        a_alt = state.delta_0 * (1 - 0.49 * sqrt(M));
                    else
                        a_alt = state.delta_0 * (1 - 0.49 * sqrt(M) ...
                                 - 3 * (state.theta_0 - obj.throttle_ratio) / (1.5 + M));
                    end

                case "mattingly_lbtf"
                    % ---- LOW-bypass mixed-flow turbofan, DRY (no afterburner) --
                    % [Mattingly, Heiser & Pratt, Aircraft Engine Design, 2nd ed.,
                    % Eq. 2.54b, "military" (maximum dry) power]
                    %
                    %   theta_0 <= TR:  alpha_mil = 0.6 delta_0
                    %   theta_0 >  TR:  alpha_mil = 0.6 delta_0 (1 - 3.8 (theta_0 - TR)/theta_0)
                    %
                    % WHY IT DIFFERS FROM THE HIGH-BYPASS FORM. There is no
                    % (1 - 0.49 sqrt(M)) term. A low-bypass engine has a small,
                    % fast exhaust jet, so its net thrust barely falls with
                    % flight speed; the ram rise in delta_0 roughly cancels the
                    % ram drag. Below TR the thrust therefore just follows the
                    % total pressure at the engine face, and at a given altitude
                    % it RISES with Mach. A high-bypass fan, with its big, slow
                    % jet, loses net thrust quickly with speed. Above TR both
                    % engines are limited by turbine temperature and throttle
                    % back - the 3.8 (theta_0 - TR)/theta_0 term.
                    %
                    % THE 0.6. Mattingly normalises this lapse by the sea-level
                    % static thrust WITH the afterburner lit, and an
                    % afterburner adds about 1/0.6 = 1.67 x the dry thrust at
                    % sea level static. This engine has no afterburner, and
                    % T_SL here is its DRY static thrust, so the 0.6 is divided
                    % out: alpha = alpha_mil / 0.6. That makes alpha = 1 at sea
                    % level static, so T_SL keeps meaning exactly what it means
                    % for the high-bypass model.
                    %
                    % Check the equation against your edition before you rely
                    % on it.
                    if state.theta_0 <= obj.throttle_ratio
                        a_alt = state.delta_0;
                    else
                        a_alt = state.delta_0 * (1 - 3.8 * (state.theta_0 - obj.throttle_ratio) / state.theta_0);
                    end

                otherwise
                    error('TfProp:UndefinedLapse', ...
                        ['Thrust lapse model "%s" is not defined. Use "mattingly_hbtf" ', ...
                         '(high bypass) or "mattingly_lbtf" (low bypass, dry).'], obj.lapse_model);
            end

            % Past the turbine-temperature limit the formula can go negative
            % (very hot, fast, low air). Thrust cannot be negative.
            a_alt = max(a_alt, 0);

            alpha = a_alt * obj.rating_factor(rating);
        end


        %% Thrust ratio at a constraint condition
        function kT = thrust_ratio(obj, state, con)
        %THRUST_RATIO  Thrust available at the condition / T_SL [-].
        %
        %   The turbofan twin of TtpaProp.power_ratio. Collects every reason
        %   the thrust at the condition differs from T_SL:
        %
        %       altitude, Mach, rating    thrust_lapse(state, rating)
        %       one engine inoperative    (n_engines - 1) / n_engines
        %       throttle setting          con.throttle
        %
        %   NOTE the OEI factor. TtpaProp used 1/n_engines (one engine of the
        %   two left running). That is the same thing for a twin, but
        %   (n-1)/n is right for ANY engine count: a trijet with one out
        %   keeps 2/3 of its thrust, not 1/3. Dividing a constraint by this
        %   factor gives Roskam's N/(N-1) multiplier on the FAR 25 climb
        %   requirements.

            if con.max_continuous
                rating = "max_continuous";
            else
                rating = "takeoff";
            end

            f_oei = 1;
            if con.oei
                % A SINGLE-engine airplane has nothing left with its engine
                % out: (n-1)/n = 0, and every constraint would divide by zero
                % and return T/W = Inf. That is not a sizing result, it is a
                % requirement that does not apply (a single-engine jet has
                % no engine-out climb gradient to meet), so stop and say so
                % instead of drawing an infinite line.
                if obj.n_engines <= 1
                    error('TfProp:OEISingleEngine', ...
                        ['Condition "%s" is one-engine-inoperative (oei = true), but the ', ...
                         'airplane has %d engine. Remove the condition from the requirements ', ...
                         'file: engine-out climb requirements do not apply to a single-engine airplane.'], ...
                        con.name, obj.n_engines);
                end
                f_oei = (obj.n_engines - 1) / obj.n_engines;
            end

            kT = obj.thrust_lapse(state, rating) * f_oei * con.throttle;
        end


        %% TSFC
        function c = tsfc(obj, state)
        %TSFC  Thrust-specific fuel consumption [1/hr].
        %
        %   "mattingly_hbtf"  [Mattingly AED 2nd ed., Eq. 3.55a]
        %       TSFC = (0.45 + 0.54 M0) sqrt(theta)       [lbm/(lbf hr)]
        %   TSFC rises with flight Mach (the engine works harder per pound
        %   of net thrust) and falls with altitude (colder air, through
        %   theta - the STATIC temperature ratio here, not theta_0).
        %
        %   "constant"  tsfc_constant everywhere - use it to reproduce a
        %   textbook hand calculation, or when you have one cruise number
        %   from an engine data sheet.
        %
        %   This replaces BSFC. Fuel flow is now c * THRUST [lbm/hr], not
        %   BSFC * shaft power.
        %
        %   CALIBRATION. The two Mattingly forms are multiplied by tsfc_scale
        %   (JSON propulsion.tsfc_scale, default 1). Set it to
        %   (published SLS TSFC of your engine) / (correlation at M = 0), so
        %   the correlation keeps its Mach and altitude TRENDS but starts from
        %   the real engine's number. Example: the T-45C file uses 0.9, which
        %   turns the low-bypass 0.90 at sea-level static into the 0.81 quoted
        %   for the Adour. 'constant' is NOT scaled - it is already a real number.
            switch obj.tsfc_model
                case "mattingly_hbtf"
                    % HIGH-bypass turbofan  [Mattingly AED 2nd ed., Eq. 3.55a]
                    c = obj.tsfc_scale * (0.45 + 0.54 * state.mach) * sqrt(state.theta);

                case "mattingly_lbtf"
                    % LOW-bypass mixed-flow turbofan, DRY (military) power
                    %   TSFC = (0.9 + 0.30 M0) sqrt(theta)     [1/hr]
                    % [Mattingly AED 2nd ed., Eq. 3.55b, military power]
                    % Roughly double the high-bypass value at low Mach: less
                    % of the thrust comes from the efficient, slow-moving bypass
                    % stream, so the engine's propulsive efficiency is lower.
                    % It rises more slowly with Mach than the high-bypass form
                    % (0.30 against 0.54), which is the other half of why low
                    % bypass suits faster airplanes. (Mattingly's afterburning
                    % form, (1.6 + 0.27 M0) sqrt(theta), is NOT included: this
                    % branch is for dry, subsonic engines.) Check the equation
                    % against your edition.
                    c = obj.tsfc_scale * (0.9 + 0.30 * state.mach) * sqrt(state.theta);

                case "constant"
                    c = obj.tsfc_constant;

                otherwise
                    error('TfProp:UndefinedTSFC', ...
                        ['TSFC model "%s" is not defined. Use "mattingly_hbtf", ', ...
                         '"mattingly_lbtf" or "constant".'], obj.tsfc_model);
            end
        end


        %% Range guard
        function T = rated_thrust_per_engine(obj)
        %RATED_THRUST_PER_ENGINE  Thrust of one engine, checked [lbf].
        %   Errors if T_SL was never set; WARNS (does not error) outside the
        %   regression's range, because an intermediate iterate of the sizing
        %   loop can leave the range and come back - same policy as
        %   TtpaProp.rated_bhp_per_engine.
            T = obj.T_engine;
            if isnan(T)
                error('TfProp:ThrustNotSet', 'T_SL has not been set yet.');
            end
            if T < min(obj.T_range) || T > max(obj.T_range)
                warning('TfProp:ThrustOutOfRange', ...
                    ['Thrust per engine (%.0f lbf) is outside the %.0f-%.0f lbf ', ...
                     'range the engine-size regression is used for.'], ...
                    T, min(obj.T_range), max(obj.T_range));
            end
        end


        %% Engine weight
        function [W_total, W_each] = engine_weight(obj)
        %ENGINE_WEIGHT  BARE (dry, uninstalled) engine weight [lbf].
        %
        %   [Raymer Eq. 10.1, non-afterburning turbofan, per engine]
        %       W = 0.084 T^1.1 exp(-0.045 BPR)
        %   with T the takeoff thrust of ONE engine [lbf]. A higher bypass
        %   ratio is LIGHTER per pound of thrust (more thrust from the fan for
        %   the same core). TfWeights multiplies by the installed-engine factor
        %   (1.3, Raymer Table 15.2 transport) - do NOT apply it here too.
        %
        %   The regression is written for UNINSTALLED catalogue thrust, so
        %   it is evaluated at uninstalled_thrust() per engine.
            if obj.is_fixed()                       % real engine: catalogue weight
                W_each  = obj.fixed_engine.dry_weight_lbf;
                W_total = obj.n_engines * W_each;
                return
            end
            T = obj.uninstalled_thrust_per_engine();
            W_each  = obj.kW * T.^obj.nW .* exp(obj.bW * obj.bypass_ratio);
            W_total = obj.n_engines * W_each;
        end


        %% Engine length
        function L = engine_length(obj)
        %ENGINE_LENGTH  Bare length of ONE engine [ft].
        %   [Raymer Eq. 10.2]   L = 0.185 T^0.4 M^0.2
        %   M is the maximum (design) Mach number: faster engines carry
        %   longer inlets and nozzles.
            if obj.is_fixed(), L = obj.fixed_engine.length_ft; return, end
            T = obj.uninstalled_thrust_per_engine();
            L = obj.kL * T.^obj.nL .* obj.design_mach.^obj.mL;
        end


        %% Engine diameter
        function D = engine_diameter(obj)
        %ENGINE_DIAMETER  Bare (fan-case) diameter of ONE engine [ft].
        %   [Raymer Eq. 10.3]   D = 0.033 T^0.5 exp(0.04 BPR)
        %   NEW compared with the TTPA: a piston engine's frontal size hardly
        %   changes with power, so the TTPA fixed the nacelle diameter. A
        %   turbofan's fan diameter grows as sqrt(thrust) (thrust ~ fan
        %   area), and more with bypass ratio, so the nacelle diameter now
        %   follows T_SL and is inside the sizing loop.
            if obj.is_fixed(), D = obj.fixed_engine.diameter_ft; return, end
            T = obj.uninstalled_thrust_per_engine();
            D = obj.kD * T.^obj.nD .* exp(obj.bD * obj.bypass_ratio);
        end


        %% Uninstalled thrust
        function T_u = uninstalled_thrust_per_engine(obj)
        %UNINSTALLED_THRUST_PER_ENGINE  Catalogue thrust of one engine [lbf].
        %   T_installed = (1 - loss) * T_uninstalled, so the catalogue figure
        %   is the installed thrust divided by (1 - loss). This is the number
        %   to compare with an engine manufacturer's data sheet.
            T_u = obj.rated_thrust_per_engine() / (1 - obj.installation_loss);
        end
    end


    methods (Access = private)
        function f = rating_factor(obj, rating)
        %RATING_FACTOR  Thrust of the rating / takeoff thrust [-].
            switch string(rating)
                case "takeoff"
                    f = 1;
                case "max_continuous"
                    f = 1 / obj.T_TO_over_T_max_continuous;
                otherwise
                    error('TfProp:UndefinedRating', ...
                        'Thrust rating "%s" is not defined.', string(rating));
            end
        end
    end
end
