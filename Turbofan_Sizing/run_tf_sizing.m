function [result, hist] = run_tf_sizing(varargin)
%RUN_TF_SIZING  Size the turbofan airplane AND its engines with the sizing loop.
%
%   run_tf_sizing
%   [result, hist] = run_tf_sizing('name', value, ...)
%
%   The turbofan counterpart of run_ttpa_sizing_mainloop. Three states are
%   iterated to a consistent set: takeoff weight W_0, installed sea-level-
%   static thrust T_SL (all engines), and wing area S_ref.
%
%     STEP 1  constraint analysis      -> least-thrust corner (W/S, T/W)
%     STEP 2  size wing and engine     S = W_0/(W/S),  T = W_0 (T/W)
%               S -> tails -> wetted area -> CD0, wave drag      TfGeom, TfAero
%               T -> engine weight, length, diameter -> nacelles TfProp, TfGeom
%     STEP 3  mission                  -> W_fuel                 mission_fuel
%     STEP 4  weight build-up          -> OEW                    TfWeights
%     STEP 5  closure                  -> W_0 new                SizingSteps
%     STEP 6  converged on W and T? otherwise relax and repeat   sizing_mainloop
%
%   Use this when the engine is a big enough share of the weight that its
%   size changes the airplane (it always is for a jet: about a fifth of OEW
%   here). For a quick thrust number at a weight you already have, use
%   run_tf_constraints.
%
%   OPTIONS (name/value, all optional; defaults from the "sizing" block)
%     'W_guess'   lbf        'S_guess'   ft^2
%     'tol'                  'max_iter'
%     'relax'                'history'   true (default) prints every pass
%     'plot'      true (default) constraint diagram + convergence plots
%     'save_png'  false (default) writes PNGs to the output folder
%     'png_prefix' start of the PNG file names (default: the JSON file name
%                 without "_requirements", e.g. "tf" or "t45c")
%     'airfoil'   "supercritical" or "conventional" (or "custom"): overrides
%                 aerodynamics.airfoil_type for this run only
%     'sweep_deg' quarter-chord sweep [deg]: overrides
%                 geometry.sweep_quarter_chord_deg for this run only (CLmax is
%                 rescaled too if the file gives CLmax_reference_sweep_deg)
%     'json'      path of a different requirements file (default
%                 tf_requirements.json) - e.g. to compare two engine types
%                 without editing the main file
%
%   OUTPUTS
%     result  the converged design, engine size included
%     hist    1xN per-pass history

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
    Sz = J.sizing;
    Cn = J.constraints;

    p = inputParser;
    p.addParameter('json',     json_path);
    p.addParameter('W_guess',  Sz.W_TO_guess_lbf);
    p.addParameter('S_guess',  Sz.S_ref_guess_ft2);
    p.addParameter('tol',      Sz.tol_rel);
    p.addParameter('max_iter', Sz.max_iter);
    p.addParameter('relax',    Sz.relaxation);
    p.addParameter('history',  true);
    p.addParameter('plot',     true);
    p.addParameter('save_png', false);
    % PNG names start with the requirements file's name minus "_requirements",
    % so tf_requirements.json writes tf_*.png and t45c_requirements.json
    % writes t45c_*.png - two aircraft never overwrite each other's figures.
    % With an override the tag is appended, e.g. prm2_conventional_35deg_*.png.
    p.addParameter('png_prefix', erase(string(json_base), "_requirements") + override_tag);
    p.addParameter('airfoil',   "");      % "supercritical" | "conventional" | "custom"
    p.addParameter('sweep_deg', []);      % quarter-chord sweep [deg]
    p.parse(varargin{:});
    A = p.Results;

    %% ------------------------------------------------------------------ build
    obj = tf_disciplines(json_path);

    % First thrust guess. Start from the design-point T/W only to put SOME
    % engine in the airplane, solve the constraints on it, and then take the
    % least-thrust corner's T/W - the same thing STEP 2 of every pass does.
    % Starting from the corner rather than the fixed design point means the
    % first airplane already meets every requirement, so the first mission
    % can always climb to altitude. (A guess that is too small in thrust
    % makes the first pass fail, and no relaxation can rescue a first pass.)
    obj.geom.S_ref = A.S_guess;
    obj.prop.T_SL  = A.W_guess * Cn.design_point.thrust_loading;
    [WS_sweep, wall] = wing_loading_sweep(obj, json_path);
    [~, ~, ~, ~, TW0] = matching_envelope(WS_sweep, obj);
    T_guess = A.W_guess * TW0;
    if obj.prop.is_fixed()
        T_guess = obj.prop.T_SL_fixed();       % a real engine: thrust is given
    end

    fprintf('\n=================== TURBOFAN SIZING LOOP ===================\n');
    fprintf('  guesses   W_0 %.0f lbf   T_SL %.0f lbf   S_ref %.1f ft^2\n', ...
            A.W_guess, T_guess, A.S_guess);
    fprintf('  sweep     %d points on [%.0f, %.0f] psf, W/S wall (tightest) at %.2f psf\n', ...
            numel(WS_sweep), WS_sweep(1), WS_sweep(end), wall);
    fprintf('  tol %.1e   max_iter %d   relaxation %.2f\n', A.tol, A.max_iter, A.relax);
    fprintf('  wing      %s airfoil (Korn kappa %.2f), sweep %.1f deg qc, CLmax factor %.3f\n', ...
            obj.aero.airfoil_type, obj.aero.kappa_A, obj.geom.sweep_qc_deg, obj.aero.CLmax_sweep_factor);
    if obj.prop.is_fixed()
        fprintf('  engine    FIXED: %d x %s, %.0f lbf uninstalled each -> T_SL %.0f lbf installed\n', ...
                obj.prop.n_engines, obj.prop.fixed_engine.name, obj.prop.fixed_engine.thrust_lbf, T_guess);
        fprintf('            the loop sizes W_TO and the wing only (smallest wing the engine allows)\n');
    end
    fprintf('  missions  %s\n', strjoin(cellfun(@(m) char(m.name), obj.missions, 'UniformOutput', false), ', '));

    %% ---------------------------------------------------------------- the loop
    % Transient passes may wander outside the engine regression's range;
    % that is checked again on the CONVERGED engine below.
    ws0 = warning;
    restore = onCleanup(@() warning(ws0));
    warning('off', 'TfProp:ThrustOutOfRange');
    warning('off', 'segment_cruise_jet:InsufficientThrust');

    lo = struct('W0', A.W_guess, 'T0', T_guess, 'S0', A.S_guess, ...
                'WS_sweep', WS_sweep, 'tol', A.tol, 'max_iter', A.max_iter, 'relax', A.relax);

    % Try the requested relaxation first, then damp harder. A rung can fail
    % in two ways: it runs out of passes (converged = false), or a pass
    % ERRORS because an over-large step produced an airplane that cannot be
    % flown (e.g. the engine cannot climb at the new weight, or the closure
    % denominator goes negative). Both are path problems, not answer
    % problems, so both move on to the next rung. Only if every rung fails
    % is the last error re-thrown - then the requirements themselves
    % probably cannot be met.
    ladder    = [A.relax, 0.25, 0.15, 0.08];
    ladder    = ladder([true, ladder(2:end) < A.relax]);   % only ever damp MORE
    converged = false;
    last_err  = [];
    for k = 1:numel(ladder)
        lo.relax = ladder(k);
        if k > 1
            fprintf('  *** retrying at relaxation %.2f\n', lo.relax);
        end
        try
            [hist, converged, state] = sizing_mainloop(obj, lo);
            if converged, break; end
        catch ME
            if strcmp(ME.identifier, 'sizing_mainloop:firstPassFailed')
                rethrow(ME);        % the guesses are the problem - stop now
            end
            last_err = ME;
            fprintf('  *** relaxation %.2f failed: %s\n', lo.relax, ME.message);
        end
    end
    if ~exist('hist', 'var')
        rethrow(last_err);
    end
    warning(ws0);

    %% ---------------------------------------------------------------- history
    if A.history
        fprintf('\n  %3s %9s %8s %7s %7s %6s %8s %7s %8s %8s %8s %9s %9s\n', ...
            '#', 'W_TO in', 'T_SL', 'S_ref', 'W/S', 'T/W', 'CD0', 'L/D max', ...
            'W_fuel', 'OEW', 'denom', 'W_TO out', 'res_W');
        fprintf('  %s\n', repmat('-', 1, 112));
        for i = 1:numel(hist)
            h = hist(i);
            fprintf('  %3d %9.1f %8.1f %7.2f %7.3f %6.4f %8.5f %7.3f %8.1f %8.1f %8.5f %9.1f %9.2e\n', ...
                h.iter, h.W_TO, h.T_SL, h.S_ref, h.WS, h.TW, h.CD0, h.LD_max, ...
                h.W_fuel, h.OEW, h.denom, h.W_TO_new, h.res_W);
        end
    end

    %% -------------------------------------------- converged design, re-evaluated
    W0 = state.W0;
    obj.prop.T_SL  = state.T0;
    obj.geom.S_ref = state.S0;
    [W_fuel, ff, fuel_seg, ~, ~, detail, msum] = mission_fuel(W0, obj);
    bd = obj.wts.OEW_breakdown(W0);
    last = hist(end);

    n    = obj.prop.n_engines;
    loss = obj.prop.installation_loss;

    result = struct();
    result.converged   = converged;
    result.n_iter      = state.n_iter;
    result.W_TO        = W0;
    result.T_SL        = state.T0;
    result.S_ref       = state.S0;
    result.WS          = last.WS;
    result.TW          = last.TW;
    result.driving     = last.driving;
    result.WS_wall     = last.WS_wall;
    result.W_fuel      = W_fuel;
    result.fuel_fraction = ff;
    result.OEW         = bd.total;
    result.OEW_breakdown = bd;
    result.OEW_statistical = obj.wts.OEW_statistical(W0);
    result.W_payload   = obj.wts.payload();
    result.CD0         = obj.aero.CD0;
    result.e           = obj.aero.e;
    result.LD_max      = obj.aero.LD_max;
    result.mission     = detail;
    result.fuel_by_segment = fuel_seg;
    result.missions    = msum;          % every mission flown; msum.governing sizes the fuel
    result.governing_mission = msum.names(msum.governing);

    % ---- ENGINE SIZE: the reason this file exists ------------------------------
    result.engine.n_engines                 = n;
    result.engine.T_installed_total_lbf     = state.T0;
    result.engine.T_installed_per_engine    = state.T0 / n;
    result.engine.T_uninstalled_per_engine  = state.T0 / n / (1 - loss);
    [result.engine.W_bare_total, result.engine.W_bare_each] = obj.prop.engine_weight();
    result.engine.length_ft                 = obj.prop.engine_length();
    result.engine.diameter_ft               = obj.prop.engine_diameter();
    result.engine.W_installed_total         = bd.installed_engine;

    %% ------------------------------------------------------------------ print
    fprintf('\n======================= CONVERGED DESIGN =======================\n');
    if converged
        fprintf('  closed in %d passes\n', state.n_iter);
    else
        fprintf('  *** DID NOT CLOSE in %d passes - values are the last iterate\n', state.n_iter);
    end
    fprintf('\n  W_TO   %9.1f lbf     S_ref %8.2f ft^2     T_SL %8.1f lbf (installed, all)\n', ...
            W0, state.S0, state.T0);
    fprintf('  W/S    %9.3f psf     T/W   %8.4f          driving: %s\n', ...
            last.WS, last.TW, last.driving);
    fprintf('  CD0    %9.5f         e     %8.4f          L/D max %.3f (subsonic)\n', ...
            result.CD0, result.e, result.LD_max);
    fprintf('  payload %8.1f lbf     fuel  %8.1f lbf     OEW %8.1f lbf\n', ...
            result.W_payload, W_fuel, bd.total);
    fprintf('  sum     %8.1f lbf against W_TO %.1f lbf\n', result.W_payload + W_fuel + bd.total, W0);

    fprintf('\n  --------------------------- ENGINE SIZE ---------------------------\n');
    fprintf('  installed SLS takeoff thrust    %8.0f lbf total   %8.0f lbf per engine\n', ...
            state.T0, state.T0 / n);
    fprintf('  UNINSTALLED, per engine         %8.0f lbf   <- compare with engine catalogues\n', ...
            result.engine.T_uninstalled_per_engine);
    if obj.prop.is_fixed()
        TW_need = last.TW_env_min;                        % least-thrust corner
        fprintf('  FIXED ENGINE %s: T/W available %.4f, least-thrust corner needs %.4f (%+.1f %% margin)\n', ...
                obj.prop.fixed_engine.name, last.TW, TW_need, 100 * (last.TW / TW_need - 1));
        fprintf('  wing set by: %s\n', last.driving);
        fprintf('  bare engine (engine data)       %8.0f lbf each, length %.2f ft, diameter %.2f ft\n', ...
                result.engine.W_bare_each, result.engine.length_ft, result.engine.diameter_ft);
    else
        fprintf('  bare engine (Raymer 10.1-10.3)  %8.0f lbf each, length %.2f ft, diameter %.2f ft\n', ...
                result.engine.W_bare_each, result.engine.length_ft, result.engine.diameter_ft);
    end
    fprintf('  installed engines (x %.2f)       %8.0f lbf total, %.1f %% of OEW\n', ...
            obj.wts.f_installed_engine, bd.installed_engine, 100 * bd.installed_engine / bd.total);
    fprintf('  nacelle  diameter %.2f ft, length %.2f ft\n', obj.geom.d_nacelle, obj.geom.l_nacelle);

    % Every mission is printed (RFP-5-P02-R04.1 wants a segment table for
    % each); the governing one - most fuel - is the one the weight closes on.
    for m = 1:numel(msum.names)
        prof = msum.profiles{m};
        tag  = "";
        if m == msum.governing, tag = "  <- GOVERNS THE FUEL"; end
        fprintf('\n  ----------------------- MISSION: %s%s\n', msum.names(m), tag);
        fprintf('  %-26s %8s %9s %7s %7s %8s %8s\n', 'segment', 'fuel', 'W end', 'CL', 'L/D', 'time hr', 'dist nm');
        fb = msum.fuel_burned{m};  det = msum.detail{m};  sw = msum.segment_weight{m};
        for i = 1:numel(det)
            sg = get_miss_seg(i, prof);
            fprintf('  %-26s %8.1f %9.1f %7.3f %7.2f %8.3f %8.1f\n', sg.name, fb(i), sw(i+1), ...
                    det(i).CL, det(i).LD, det(i).time_hr, det(i).dist_nm);
        end
        fprintf('  %-26s %8.1f   (x (1 + %.3f reserve) = %.1f lbf)   time %.2f hr\n', 'total burned', ...
                sum(fb), prof.reserve_fuel_fraction, msum.W_fuel(m), sum([det.time_hr], 'omitnan'));
    end

    fprintf('\n  -------------------------- WEIGHT BUILD-UP --------------------------\n');
    if isfield(bd, 'v_tail')
        tailrows = {'V-tail (both panels)', bd.v_tail};
    else
        tailrows = {'horizontal tail', bd.horizontal_tail; 'vertical tail', bd.vertical_tail};
    end
    rows = [{'wing', bd.wing}; tailrows; ...
            {'fuselage', bd.fuselage; ...
             'landing gear', bd.landing_gear; 'installed engines', bd.installed_engine; ...
             'all else empty', bd.all_else_empty}];
    for i = 1:size(rows, 1)
        fprintf('  %-20s %9.1f lbf  %5.1f %%\n', rows{i,1}, rows{i,2}, 100 * rows{i,2} / bd.total);
    end
    fprintf('  %-20s %9.1f lbf\n', 'OEW (build-up)', bd.total);
    fprintf('  %-20s %9.1f lbf  (%+.1f %% vs build-up; Raymer Table 3.1 trend in the JSON)\n', ...
            'OEW (statistical)', result.OEW_statistical, ...
            100 * (result.OEW_statistical - bd.total) / bd.total);

    %% ----------------------------------------------------------------- checks
    fprintf('\n  ------------------------ POST-SIZING CHECKS ------------------------\n');
    [ok, V_ft3, W_cap, margin] = wing_fuel_check(W_fuel, obj);
    if ok
        fprintf('  wing fuel volume %.1f ft^3 holds %.0f lbf: OK, %.1f %% spare\n', V_ft3, W_cap, 100*margin);
    else
        fprintf('  *** wing holds %.0f lbf, mission needs %.0f lbf: SHORT by %.1f %%\n', ...
                W_cap, W_fuel, -100*margin);
        fprintf('      (business jets often carry the rest in a fuselage tank)\n');
    end
    [Pn, Pm, Ps] = landing_gear_loads(W0, obj);
    fprintf('  gear static loads: nose %.0f lbf, main %.0f lbf (%.0f per strut)\n', Pn, Pm, Ps);
    Tr = obj.prop.T_range;
    if result.engine.T_uninstalled_per_engine < Tr(1) || result.engine.T_uninstalled_per_engine > Tr(2)
        fprintf('  *** engine thrust is outside the %.0f-%.0f lbf regression range\n', Tr(1), Tr(2));
    end
    ic = find(strcmp([detail.type], "cruise"), 1);
    if ~isempty(ic)
        Mdd = obj.aero.drag_divergence_mach(detail(ic).CL);
        fprintf('  Korn drag-divergence Mach at the mean cruise CL: %.3f\n', Mdd);
    end

    %% ------------------------------------------------------------------- plots
    if A.plot
        out = fullfile(fileparts(mfilename('fullpath')), 'output');
        o = struct('design_point', [last.WS, last.TW], ...
                   'title', sprintf('Constraint diagram of the converged airplane, W_{TO} = %.0f lbf', W0));
        if A.save_png, o.save_png = fullfile(out, A.png_prefix + "_converged_constraint_diagram.png"); end
        plot_constraint_diagram(obj, WS_sweep, o);

        f = figure('Name', 'Turbofan sizing loop', 'Color', 'w', 'Position', [120 120 1100 650]);
        it = [hist.iter];
        subplot(2,2,1); plot(it, [hist.W_TO], '-o', 'MarkerSize', 3); grid on;
        xlabel('pass'); ylabel('W_{TO} [lbf]'); title('Takeoff weight');
        subplot(2,2,2); plot(it, [hist.T_SL], '-o', 'MarkerSize', 3); grid on;
        xlabel('pass'); ylabel('T_{SL} [lbf]'); title('Installed SLS thrust (all engines)');
        subplot(2,2,3); plot(it, [hist.S_ref], '-o', 'MarkerSize', 3); grid on;
        xlabel('pass'); ylabel('S_{ref} [ft^2]'); title('Wing area');
        subplot(2,2,4);
        semilogy(it, max([hist.res_W], eps), '-o', it, max([hist.res_T], eps), '-s', ...
                 it, max([hist.res_S], eps), '-^', 'MarkerSize', 3); grid on;
        legend({'W', 'T', 'S'}); xlabel('pass'); ylabel('relative residual'); title('Convergence');
        if A.save_png
            if ~exist(out, 'dir'), mkdir(out); end
            exportgraphics(f, fullfile(out, A.png_prefix + "_sizing_convergence.png"), 'Resolution', 200);
        end
    end

    result.history = hist;
    result.obj     = obj;      % the converged airplane (geometry, aero, engine, weights)
    fprintf('\nDone.\n');
end
