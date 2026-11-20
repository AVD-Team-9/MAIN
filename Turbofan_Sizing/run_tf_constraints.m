function R = run_tf_constraints(varargin)
%RUN_TF_CONSTRAINTS  Constraint diagram and engine thrust at a GIVEN takeoff weight.
%
%   R = run_tf_constraints
%   R = run_tf_constraints('W_TO', 15000, 'design_point', [62 0.38], 'plot', true)
%
%   The quick engine-sizing answer: "for an airplane of this weight, how much
%   thrust do the requirements demand?". It does NOT close the weight - for
%   that, run run_tf_sizing, which iterates W_TO, T_SL and S_ref together.
%
%   OPTIONS (name/value, all optional)
%     'W_TO'          takeoff weight [lbf]; default constraints.W_TO_estimate_lbf
%     'design_point'  [W/S, T/W] you have chosen; default constraints.design_point
%     'plot'          true (default) draws the constraint diagram
%     'save_png'      true (default false) writes output/<prefix>_constraint_diagram.png
%     'png_prefix'    default: the JSON file name without "_requirements"
%     'json'          path of a different requirements file (default
%                     tf_requirements.json)
%     'airfoil'       "supercritical" or "conventional" (or "custom"):
%                     overrides aerodynamics.airfoil_type for this run only
%     'sweep_deg'     quarter-chord sweep [deg]: overrides
%                     geometry.sweep_quarter_chord_deg for this run only
%                     (CLmax is rescaled too if the file gives
%                     aerodynamics.CLmax_reference_sweep_deg)
%
%   WHAT IT DOES
%   1. Builds the bundle and puts a first airplane in it (S_ref from the
%      design-point W/S, T_SL from the design-point T/W).
%   2. Repeats a few times: solve the constraints on that airplane, move it
%      to the least-thrust corner (S_ref = W_TO/(W/S), T_SL = W_TO (T/W)).
%      This short fixed-weight iteration is needed because the drag (and so
%      the cruise and ceiling curves) depends on S_wet/S_ref, and the
%      nacelle size depends on T_SL - the diagram belongs to an airplane.
%   3. Prints every requirement at the least-thrust corner and at your
%      design point, with the margin of each.
%   4. Prints the ENGINE SIZE: installed and uninstalled thrust, total and
%      per engine, and the bare engine weight, length and diameter.
%   5. Draws the diagram.
%
%   OUTPUT  R  struct with the corner, the design point, the thrusts and the
%              diagram data (R.diagram).

    %% ---------------------------------------------------------------- options
    % 'json' is read BEFORE the parser, because the other defaults come
    % from whichever requirements file is in use.
    json_path = tf_requirements_path();
    airfoil = "";  sweep_deg = [];
    for i = 1:2:numel(varargin) - 1
        if strcmpi(varargin{i}, 'json'),      json_path = string(varargin{i+1}); end
        if strcmpi(varargin{i}, 'airfoil'),   airfoil   = string(varargin{i+1}); end
        if strcmpi(varargin{i}, 'sweep_deg'), sweep_deg = varargin{i+1};         end
    end
    % 'airfoil' / 'sweep_deg' overrides: run on a modified COPY of the file
    % (tf_json_with_overrides), so every class sees the same wing. The
    % original requirements file is never changed.
    [~, json_base] = fileparts(json_path);
    [json_path, override_tag] = tf_json_with_overrides(json_path, airfoil, sweep_deg);
    J  = jsondecode(fileread(json_path));
    Cn = J.constraints;

    p = inputParser;
    p.addParameter('json', json_path);
    p.addParameter('W_TO', Cn.W_TO_estimate_lbf, @(x) isscalar(x) && x > 0);
    p.addParameter('design_point', [Cn.design_point.wing_loading_psf, ...
                                    Cn.design_point.thrust_loading], @(x) isnumeric(x) && numel(x) == 2);
    p.addParameter('plot', true);
    p.addParameter('save_png', false);
    % PNG names start with the requirements file's name minus "_requirements",
    % so tf_requirements.json writes tf_*.png and t45c_requirements.json
    % writes t45c_*.png - two aircraft never overwrite each other's figures.
    % With an override the tag is appended, e.g. prm2_conventional_35deg_*.png.
    p.addParameter('png_prefix', erase(string(json_base), "_requirements") + override_tag);
    p.addParameter('airfoil',   "");      % "supercritical" | "conventional" | "custom"
    p.addParameter('sweep_deg', []);      % quarter-chord sweep [deg]
    p.parse(varargin{:});
    W_TO = p.Results.W_TO;
    dp   = p.Results.design_point;

    %% ------------------------------------------------------- 1. the airplane
    obj = tf_disciplines(json_path);
    fixed = obj.prop.is_fixed();
    if fixed
        % A REAL engine: the thrust is given, so the design point's T/W is
        % whatever that engine gives at this weight (the JSON value is ignored).
        dp(2) = obj.prop.T_SL_fixed() / W_TO;
    end
    obj.geom.S_ref = W_TO / dp(1);
    obj.prop.T_SL  = W_TO * dp(2);

    %% ------------------------------- 2. move the airplane to its own corner
    % Silence the regression-range warning while the airplane is moving.
    ws0 = warning('off', 'TfProp:ThrustOutOfRange');
    restore = onCleanup(@() warning(ws0));

    for it = 1:20
        WS = wing_loading_sweep(obj, json_path);
        [TW_env, WS_wall, ~, WS_best, TW_best] = matching_envelope(WS, obj);
        if isnan(WS_best)
            error('run_tf_constraints:NoFeasiblePoint', ...
                'No wing loading on the sweep is feasible (wall at %.2f psf).', WS_wall);
        end
        if fixed
            % Fixed engine: the airplane on the diagram keeps the real engine
            % and takes the smallest wing it allows (fixed_engine_wing_loading).
            [WS_fix, ~, fix_limit, fix_ok] = fixed_engine_wing_loading( ...
                WS, TW_env, WS_wall, obj.prop.T_SL_fixed(), W_TO, obj);
            S_new = W_TO / WS_fix;
            T_new = obj.prop.T_SL_fixed();
        else
            S_new = W_TO / WS_best;
            T_new = W_TO * TW_best;
        end
        change = max(abs(S_new - obj.geom.S_ref) / S_new, abs(T_new - obj.prop.T_SL) / T_new);
        obj.geom.S_ref = S_new;
        obj.prop.T_SL  = T_new;
        if change < 1e-6, break; end
    end
    warning(ws0);

    %% ------------------------------------------------ 3. every requirement
    [TW_lim, walls, names] = run_constraints([WS_best, dp(1)], obj);

    fprintf('\n=============== TURBOFAN CONSTRAINT ANALYSIS (fixed W_TO) ===============\n');
    fprintf('  W_TO %.0f lbf  (given, NOT closed - run run_tf_sizing for that)\n', W_TO);
    fprintf('  airplane on the diagram:  S_ref %.1f ft^2,  T_SL %.0f lbf,  CD0 %.5f\n', ...
            obj.geom.S_ref, obj.prop.T_SL, obj.aero.CD0);
    fprintf('  wing: %s airfoil (Korn kappa %.2f), quarter-chord sweep %.1f deg, LE %.1f deg\n', ...
            obj.aero.airfoil_type, obj.aero.kappa_A, obj.geom.sweep_qc_deg, obj.geom.sweep_LE_deg);
    fprintf('        CLmax clean/TO/landing %.3f / %.3f / %.3f (sweep factor %.3f)\n', ...
            obj.aero.CLmax_clean, obj.aero.CLmax_takeoff, obj.aero.CLmax_landing, obj.aero.CLmax_sweep_factor);
    fprintf('  fixed-weight iteration closed in %d steps\n\n', it);

    fprintf('  %-32s %14s %14s %10s\n', 'requirement', 'T/W at corner', 'T/W at design', 'margin');
    fprintf('  %s\n', repmat('-', 1, 74));
    for k = 1:numel(names)
        if isnan(walls(k))
            % margin = how much more T/W the design point has than this needs
            fprintf('  %-32s %14.4f %14.4f %9.1f%%\n', names(k), TW_lim(k,1), TW_lim(k,2), ...
                    100 * (dp(2) - TW_lim(k,2)) / TW_lim(k,2));
        else
            fprintf('  %-32s %14s %14s %9.1f%%   (W/S wall at %.2f psf)\n', names(k), ...
                    '-', '-', 100 * (walls(k) - dp(1)) / walls(k), walls(k));
        end
    end

    [~, idx] = max(TW_lim(:,1));
    fprintf('\n  least-thrust corner   W/S %.2f psf   T/W %.4f   driven by %s\n', ...
            WS_best, TW_best, names(idx));
    fprintf('  your design point     W/S %.2f psf   T/W %.4f\n', dp(1), dp(2));
    if dp(1) > WS_wall
        fprintf('  *** the design point is BEYOND the %.2f psf wing-loading wall\n', WS_wall);
    end
    if any(TW_lim(isnan(walls), 2) > dp(2))
        fprintf('  *** the design point does NOT meet every thrust requirement\n');
    end
    if fixed
        fprintf('\n  FIXED ENGINE %s: T/W at this weight %.4f (design-point T/W is set to it)\n', ...
                obj.prop.fixed_engine.name, dp(2));
        if fix_ok
            fprintf('  smallest wing it allows: W/S %.2f psf, S_ref %.1f ft^2 (limited by %s)\n', ...
                    WS_fix, W_TO / WS_fix, fix_limit);
            fprintf('  thrust margin over the least-thrust corner: %+.1f %%\n', 100 * (dp(2) / TW_best - 1));
        else
            fprintf('  *** %s\n', fix_limit);
        end
    end

    %% ------------------------------------------------------ 4. engine size
    n = obj.prop.n_engines;
    loss = obj.prop.installation_loss;
    T_corner = TW_best * W_TO;
    T_design = dp(2)  * W_TO;

    fprintf('\n------------------------------- ENGINE SIZE -------------------------------\n');
    fprintf('  T_SL = (T/W) x W_TO, installed sea-level static takeoff thrust\n\n');
    fprintf('  %-28s %12s %12s\n', '', 'corner', 'design pt');
    fprintf('  %-28s %12.0f %12.0f  lbf\n', 'installed, all engines', T_corner, T_design);
    fprintf('  %-28s %12.0f %12.0f  lbf\n', 'installed, per engine', T_corner/n, T_design/n);
    fprintf('  %-28s %12.0f %12.0f  lbf   (/ (1 - %.2f) installation loss)\n', ...
            'UNINSTALLED, per engine', T_corner/n/(1-loss), T_design/n/(1-loss), loss);

    % Engine dimensions at the DESIGN-POINT thrust
    obj.prop.T_SL = T_design;
    [Wb_tot, Wb_each] = obj.prop.engine_weight();
    if fixed
        fprintf('\n  fixed engine (engine data, per engine):\n');
    else
        fprintf('\n  at the design-point thrust (Raymer Eqs. 10.1-10.3, per engine):\n');
    end
    fprintf('    bare weight %.0f lbf (%.0f lbf for %d)   length %.2f ft   diameter %.2f ft\n', ...
            Wb_each, Wb_tot, n, obj.prop.engine_length(), obj.prop.engine_diameter());
    fprintf('  Compare the UNINSTALLED per-engine thrust with engine catalogues to pick\n');
    fprintf('  a candidate, then re-run with that engine''s lapse and TSFC.\n');

    %% -------------------------------------------------------------- 5. plot
    % Plot the airplane the curves were solved for: the corner airplane, or
    % for a fixed engine the real engine with its smallest wing.
    obj.prop.T_SL = T_corner;
    if fixed, obj.prop.T_SL = obj.prop.T_SL_fixed(); end
    D = [];
    if p.Results.plot
        o = struct('design_point', dp, ...
                   'title', sprintf('Constraint diagram at W_{TO} = %.0f lbf', W_TO));
        if p.Results.save_png
            o.save_png = fullfile(fileparts(mfilename('fullpath')), 'output', p.Results.png_prefix + "_constraint_diagram.png");
        end
        [~, D] = plot_constraint_diagram(obj, WS, o);
    end

    R = struct('W_TO', W_TO, 'WS_best', WS_best, 'TW_best', TW_best, ...
               'WS_wall', WS_wall, 'design_point', dp, ...
               'T_SL_corner', T_corner, 'T_SL_design', T_design, ...
               'T_per_engine_uninstalled_design', T_design/n/(1-loss), ...
               'S_ref', W_TO / WS_best, 'diagram', D, 'obj', obj);
end
