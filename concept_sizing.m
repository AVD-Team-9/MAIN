function [result, hist] = concept_sizing(json_path, varargin)
%CONCEPT_SIZING  Size the takeoff weight of a FIXED-GEOMETRY concept, then check every requirement.
%
%   [result, hist] = concept_sizing('concept5_team.json')
%   [result, hist] = concept_sizing(f, 'S_ref', 204.1, 'save_png', true)
%
%   The concept-analysis counterpart of Turbofan_Sizing/run_tf_sizing, and it
%   reads the SAME JSON format. The difference is what is held fixed:
%
%                      run_tf_sizing (design)          concept_sizing (analysis)
%       wing area      chosen every pass (W/S)         FIXED (geometry.S_ref_fixed_ft2)
%       other geometry fixed shape, scales with S      fixed (S does not change)
%       engine         rubber or fixed                 rubber or fixed (see below)
%       W_TO           closed by the loop              closed by the loop
%       requirements   put the design point ON the     CHECKED at the design point,
%                      feasible-region boundary        each with a margin
%
%   THE LOOP (one pass, per Raymer's sizing logic):
%     1. Airplane: S_ref fixed; thrust fixed (real engine), or - for a rubber
%        engine - the least thrust that meets every requirement at the
%        concept's W/S = W_0/S_ref.
%     2. Missions: every mission in the file flown at W_0; the hungriest one
%        sets the fuel (Turbofan_Sizing/mission_fuel).
%     3. Empty weight: the component build-up at W_0 (wing, tails and
%        fuselage are now constant; gear, all-else and fuel vary with W_0).
%     4. Closure: W_new = W_payload / (1 - W_fuel/W_0 - OEW/W_0).
%     5. Solve f(W_0) = W_0 - payload - fuel - OEW = 0 for W_0 (bracketed
%        regula falsi - one unknown, so no relaxation factor to tune).
%   Then concept_constraint_check evaluates every requirement at the
%   converged W/S and T/W and finds the HEAVIEST weight the concept could
%   reach before a requirement fails (weight-growth margin).
%
%   A requirement that FAILS does not stop the run - the concept is simply
%   reported as not meeting it, with its margin. Only a mission the airplane
%   cannot fly (e.g. it cannot climb to cruise altitude) stops the loop.
%
%   INPUT. A Turbofan_Sizing requirements file plus one key:
%       geometry.S_ref_fixed_ft2   the concept's reference wing area [ft^2]
%   (or pass 'S_ref'). Files in inputs/ are found by name.
%
%   OPTIONS (name/value)
%     'S_ref'      overrides geometry.S_ref_fixed_ft2 [ft^2]
%     'W_guess'    default sizing.W_TO_guess_lbf
%     'tol'        default sizing.tol_rel        'max_iter' default sizing.max_iter
%     (sizing.relaxation is not used: the closure is a 1-D root-find, see closure_)
%     'history'    true (default) prints every pass
%     'plot'       true (default): constraint diagram with the design point
%                  and the weight-growth line
%     'save_png'   false (default); true writes output/<name>_concept_diagram.png
%
%   OUTPUT  result struct: W_TO, S_ref, WS, TW, T_SL, W_fuel, OEW,
%           OEW_breakdown, missions, check (concept_constraint_check), obj
%           hist   per-pass history

    json_path = concept_setup(json_path);
    J = jsondecode(fileread(json_path));

    p = inputParser;
    S_default = [];
    if isfield(J.geometry, 'S_ref_fixed_ft2'), S_default = J.geometry.S_ref_fixed_ft2; end
    p.addParameter('S_ref',    S_default);
    p.addParameter('W_guess',  J.sizing.W_TO_guess_lbf);
    p.addParameter('tol',      J.sizing.tol_rel);
    p.addParameter('max_iter', J.sizing.max_iter);
    p.addParameter('relax',    J.sizing.relaxation);
    p.addParameter('history',  true);
    p.addParameter('plot',     true);
    p.addParameter('save_png', false);
    p.parse(varargin{:});
    A = p.Results;
    if isempty(A.S_ref)
        error('concept_sizing:NoWingArea', ['%s has no geometry.S_ref_fixed_ft2. Add the concept''s ', ...
            'reference wing area there, or pass ''S_ref''.'], json_path);
    end
    S = A.S_ref;

    %% ------------------------------------------------------------- the airplane
    obj = tf_disciplines(json_path);
    obj.geom.S_ref = S;                          % FIXED for the whole run
    fixed = obj.prop.is_fixed();
    if fixed
        obj.prop.T_SL = obj.prop.T_SL_fixed();
        eng = sprintf('FIXED %s, T_SL %.0f lbf installed', obj.prop.fixed_engine.name, obj.prop.T_SL);
    else
        obj.prop.T_SL = A.W_guess * 0.4;         % first guess; resized every pass
        eng = 'RUBBER: least thrust meeting every requirement at the concept''s W/S';
    end
    [~, nm] = fileparts(json_path);

    fprintf('\n===================== CONCEPT ANALYSIS (fixed geometry) =====================\n');
    fprintf('  file      %s\n', json_path);
    fprintf('  wing      S_ref %.2f ft^2 FIXED, AR %.3f, span %.2f ft, taper %.3f, sweep %.2f deg qc, %s airfoil\n', ...
            S, obj.geom.AR, obj.geom.b, obj.geom.taper, obj.geom.sweep_qc_deg, obj.aero.airfoil_type);
    fprintf('  tails     S_ht %.1f ft^2, S_vt %.1f ft^2   fuselage %.1f ft   S_wet %.1f ft^2   CD0 %.5f   e %.3f\n', ...
            obj.geom.S_ht, obj.geom.S_vt, obj.geom.l_fus, obj.geom.S_wet, obj.aero.CD0, obj.aero.e);
    if isprop(obj.geom, 'tail_type') && obj.geom.tail_type == "v_tail"
        g = obj.geom;
        fprintf(['  V-tail    S_V %.2f ft^2 at %.1f deg dihedral; projected %.2f / %.2f ft^2 ', ...
                 '(required %.2f / %.2f); b_V %.2f ft, projected %.2f ft\n'], ...
                g.S_vtail, g.vtail_dihedral_deg, g.S_ht, g.S_vt, g.S_ht_required, g.S_vt_required, ...
                g.b_vtail, g.b_vtail_projected);
    end
    if isprop(obj.geom, 'tail_type') && obj.geom.tail_type == "twin_vertical"
        g = obj.geom;
        fprintf(['  fins      %d x %.2f ft^2 canted %.1f deg (true %.2f ft^2); yaw-effective %.2f ft^2 ', ...
                 '(required %.2f); fin span %.2f ft, MAC %.2f ft\n'], ...
                g.n_fins, g.S_fin, g.fin_cant_deg, g.S_vt, g.S_vt_projected, g.S_vt_required, g.b_vt, g.MAC_vt);
    end
    if obj.aero.trim.enabled
        CLc = [0.3 1.0];
        fprintf('  trim      Cm0_w %.4f; tail CL_h %.4f / %.4f, trim dCD %.5f / %.5f at CL %.1f / %.1f\n', ...
                obj.aero.wing_cm0(), obj.aero.trim_tail_CL(CLc), obj.aero.trim_drag(CLc), CLc);
    end
    fprintf('  engine    %s\n', eng);
    fprintf('  missions  %s\n', strjoin(cellfun(@(m) char(m.name), obj.missions, 'UniformOutput', false), ', '));

    %% ---------------------------------------------------------------- the loop
    ws0 = warning;  restore = onCleanup(@() warning(ws0));
    warning('off', 'TfProp:ThrustOutOfRange');
    warning('off', 'segment_cruise_jet:InsufficientThrust');

    [hist, converged, W0] = closure_(obj, S, fixed, A.W_guess, A.tol, A.max_iter);

    if A.history
        fprintf('\n  %3s %10s %9s %9s %9s %10s\n', '#', 'W_TO', 'T_SL', 'W_fuel', 'OEW', 'residual');
        for i = 1:numel(hist)
            h = hist(i);
            fprintf('  %3d %10.1f %9.1f %9.1f %9.1f %10.2f\n', i, h.W_TO, h.T_SL, h.W_fuel, h.OEW, h.f);
        end
    end

    %% -------------------------------------------- converged airplane, re-evaluated
    set_thrust_(obj, W0, S, fixed);
    [W_fuel, ff, ~, ~, ~, detail, msum] = mission_fuel(W0, obj);
    obj.wts.W_TO = W0;  obj.wts.W_energy = W_fuel;
    bd = obj.wts.OEW_breakdown(W0);
    C  = concept_constraint_check(obj, W0);

    result = struct('converged', converged, 'n_iter', numel(hist), 'W_TO', W0, 'S_ref', S, ...
        'WS', W0 / S, 'TW', obj.prop.T_SL / W0, 'T_SL', obj.prop.T_SL, ...
        'T_uninstalled_per_engine', obj.prop.T_SL / obj.prop.n_engines / (1 - obj.prop.installation_loss), ...
        'W_fuel', W_fuel, 'fuel_fraction', ff, 'OEW', bd.total, 'OEW_breakdown', bd, ...
        'W_payload', obj.wts.payload(), 'CD0', obj.aero.CD0, 'e', obj.aero.e, ...
        'missions', msum, 'governing_mission', msum.names(msum.governing), 'mission', detail, ...
        'check', C, 'obj', obj, 'json', json_path);

    %% ------------------------------------------------------------------- print
    fprintf('\n=========================== CONVERGED CONCEPT ===========================\n');
    if converged, fprintf('  closed in %d evaluations\n', numel(hist));
    else, fprintf('  *** DID NOT CLOSE - values are the last iterate\n'); end
    fprintf('\n  W_TO   %9.1f lbf     S_ref %8.2f ft^2 (fixed)   W/S %7.2f psf\n', W0, S, result.WS);
    fprintf('  T_SL   %9.1f lbf     T/W   %8.4f                %s\n', obj.prop.T_SL, result.TW, ...
            ternary_(fixed, '(fixed engine)', '(rubber engine sized at this W/S)'));
    fprintf('  payload %8.1f lbf     fuel  %8.1f lbf (%s)   OEW %8.1f lbf\n', result.W_payload, W_fuel, ...
            result.governing_mission, bd.total);
    fprintf('  sum     %8.1f lbf against W_TO %.1f lbf\n', result.W_payload + W_fuel + bd.total, W0);

    fprintf('\n  --------------------------- REQUIREMENTS ---------------------------\n');
    fprintf('  %-34s %10s %10s %9s  %s\n', 'requirement', 'needed', 'available', 'margin', '');
    T = C.table;
    for k = 1:height(T)
        if T.type(k) == "landing" || T.type(k) == "approach_speed" || T.type(k) == "instantaneous_turn"
            unit = 'psf max';
        else
            unit = 'T/W';
        end
        fprintf('  %-34s %10.4f %10.4f %+8.1f%%  %s  (%s)\n', T.requirement(k), T.required(k), T.available(k), ...
                T.margin_pct(k), ternary_(T.pass(k), 'PASS', '*** FAIL'), unit);
    end
    if C.all_pass
        fprintf('  ALL REQUIREMENTS MET. Smallest margin: %s.\n', C.worst);
    else
        fprintf('  *** %d requirement(s) NOT met.\n', sum(~T.pass));
    end
    if fixed
        if isnan(C.W_max)
            fprintf('  weight-growth margin: none - no weight in [0.4, 3] x W_TO meets every requirement\n');
        else
            fprintf('  heaviest W_TO meeting every requirement: %.0f lbf (%+.1f %% vs W_TO), limited by %s\n', ...
                    C.W_max, C.growth_pct, C.W_max_limit);
        end
    end

    for m = 1:numel(msum.names)
        prof = msum.profiles{m};
        tag = "";  if m == msum.governing, tag = "  <- GOVERNS THE FUEL"; end
        fprintf('\n  ----------------------- MISSION: %s%s\n', msum.names(m), tag);
        fprintf('  %-26s %8s %9s %7s %7s %8s %8s\n', 'segment', 'fuel', 'W end', 'CL', 'L/D', 'time hr', 'dist nm');
        fb = msum.fuel_burned{m};  det = msum.detail{m};  sw = msum.segment_weight{m};
        for i = 1:numel(det)
            sg = get_miss_seg(i, prof);
            fprintf('  %-26s %8.1f %9.1f %7.3f %7.2f %8.3f %8.1f\n', sg.name, fb(i), sw(i+1), ...
                    det(i).CL, det(i).LD, det(i).time_hr, det(i).dist_nm);
        end
        fprintf('  %-26s %8.1f   (x (1 + %.3f reserve) = %.1f lbf)\n', 'total burned', sum(fb), ...
                prof.reserve_fuel_fraction, msum.W_fuel(m));
    end

    fprintf('\n  -------------------------- WEIGHT BUILD-UP --------------------------\n');
    if isfield(bd, 'v_tail')
        tailrows = {'V-tail (both panels)', bd.v_tail};
    else
        tailrows = {'horizontal tail', bd.horizontal_tail; 'vertical tail', bd.vertical_tail};
    end
    rows = [{'wing', bd.wing}; tailrows; ...
            {'fuselage', bd.fuselage; 'landing gear', bd.landing_gear; 'installed engines', bd.installed_engine; ...
             'all else empty', bd.all_else_empty}];
    for i = 1:size(rows, 1)
        fprintf('  %-20s %9.1f lbf  %5.1f %%\n', rows{i,1}, rows{i,2}, 100 * rows{i,2} / bd.total);
    end
    fprintf('  %-20s %9.1f lbf\n', 'OEW (build-up)', bd.total);

    fprintf('\n  ------------------------------ CHECKS ------------------------------\n');
    [okf, V_ft3, W_cap, margin] = wing_fuel_check(W_fuel, obj);
    if okf
        fprintf('  wing fuel volume %.1f ft^3 holds %.0f lbf: OK, %.1f %% spare\n', V_ft3, W_cap, 100*margin);
    else
        fprintf('  *** wing holds %.0f lbf, mission needs %.0f lbf: SHORT by %.1f %% (fuselage tanks needed)\n', ...
                W_cap, W_fuel, -100*margin);
    end
    ic = find(strcmp([detail.type], "cruise"), 1);
    if ~isempty(ic)
        fprintf('  Korn drag-divergence Mach at the mean cruise CL: %.3f\n', obj.aero.drag_divergence_mach(detail(ic).CL));
    end

    %% ------------------------------------------------------------------- plot
    if A.plot
        WS = wing_loading_sweep(obj, json_path);
        WS = unique([WS, result.WS]);
        o = struct('design_point', [result.WS, result.TW], ...
                   'title', sprintf('%s: fixed S_{ref} %.0f ft^2, W_{TO} %.0f lbf', strrep(nm, '_', '\_'), S, W0), ...
                   'TW_max', min(1.2, max(0.6, 1.4 * result.TW)));
        [fig, ~] = plot_constraint_diagram(obj, WS, o);
        concept_growth_line_(fig.CurrentAxes, result, fixed);
        if (islogical(A.save_png) && A.save_png) || (~islogical(A.save_png) && strlength(string(A.save_png)) > 0)
            if islogical(A.save_png)
                out = fullfile(fileparts(mfilename('fullpath')), 'output', nm + "_concept_diagram.png");
            else
                out = string(A.save_png);
            end
            if ~exist(fileparts(out), 'dir'), mkdir(fileparts(out)); end
            exportgraphics(fig, out, 'Resolution', 200);
            fprintf('  wrote %s\n', out);
        end
    end
    fprintf('\nDone.\n');
end

%% ===========================================================================
function [hist, converged, W0] = closure_(obj, S, fixed, W0, tol, max_iter)
%CLOSURE_  Solve the weight closure for W_TO with the geometry held constant.
%
%   With the wing (and a real engine) fixed, W_TO is the ONLY unknown, so
%   the closure is a 1-D root-finding problem:
%
%       f(W) = W - W_payload - W_fuel(W) - OEW(W) = 0
%
%   f is the weight left over at a trial W: negative means the trial airplane
%   cannot carry its own fuel and empty weight, positive means it has room
%   to spare. f rises with W when a solution exists (each extra pound of
%   W_TO needs less than a pound of extra fuel and structure).
%
%   WHY NOT UNDER-RELAXATION (as run_tf_sizing does). With the wing fixed,
%   wing and tail weight no longer grow with W_TO, and the fixed-point map
%   W -> W_payload / (1 - W_f/W - OEW/W) becomes very steep near the answer:
%   relaxation factors that work in the design loop then overshoot into
%   W_TO values where the closure has no solution, or oscillate. A
%   bracketed root-finder converges in a handful of evaluations, with no
%   tuning:
%     1. bracket: step W up or down by 10 % from W_guess until f changes sign;
%     2. Illinois regula falsi inside the bracket until |f| / W < tol.
%   A trial weight whose mission cannot be flown (e.g. the airplane cannot
%   climb to cruise altitude) is reported with the reason.
    hist = struct('W_TO', {}, 'T_SL', {}, 'W_fuel', {}, 'OEW', {}, 'f', {});
    converged = false;
    pay = obj.wts.payload();
    function v = f_(W)
        set_thrust_(obj, W, S, fixed);
        Wf = mission_fuel(W, obj);
        obj.wts.W_TO = W;  obj.wts.W_energy = Wf;
        bd = obj.wts.OEW_breakdown(W);
        v  = W - pay - Wf - bd.total;
        hist(end + 1) = struct('W_TO', W, 'T_SL', obj.prop.T_SL, 'W_fuel', Wf, 'OEW', bd.total, 'f', v);
    end

    % ---- 1. bracket the root -----------------------------------------------
    try
        fa = f_(W0);
    catch ME
        error('concept_sizing:firstPassFailed', 'The mission cannot be flown at W_guess = %.0f lbf: %s  Try another ''W_guess''.', ...
              W0, ME.message);
    end
    a = W0;  b = NaN;  fb = NaN;
    step = 1.10;  if fa > 0, step = 1 / 1.10; end   % too much room -> try lighter
    for k = 1:40
        Wt = a * step;
        try
            ft = f_(Wt);
        catch ME
            error('concept_sizing:CannotBracket', ['Searching for W_TO, the mission could not be flown at ', ...
                  '%.0f lbf: %s'], Wt, ME.message);
        end
        if sign(ft) ~= sign(fa)
            b = Wt;  fb = ft;  break
        end
        a = Wt;  fa = ft;
        if Wt > 1e6 || Wt < 500, break, end
    end
    if isnan(b)
        error('concept_sizing:NoClosure', ['No takeoff weight closes: the leftover weight W - payload - fuel - OEW ', ...
              'never changes sign (last %.0f lb at W_TO %.0f lbf). Fuel plus empty weight grow faster than W_TO - ', ...
              'the concept cannot carry its missions.'], fa, a);
    end

    % ---- 2. Illinois regula falsi ---------------------------------------------
    side = 0;
    for it = 1:max_iter
        c  = (a * fb - b * fa) / (fb - fa);
        fc = f_(c);
        if abs(fc) / c < tol
            W0 = c;  converged = true;  return
        end
        if sign(fc) == sign(fb)
            b = c;  fb = fc;
            if side == -1, fa = fa / 2; end
            side = -1;
        else
            a = c;  fa = fc;
            if side == +1, fb = fb / 2; end
            side = +1;
        end
    end
    W0 = c;
    warning('concept_sizing:NotConverged', 'Closure did not converge in %d steps (residual %.1f lb).', max_iter, fc);
end

function set_thrust_(obj, W0, S, fixed)
%SET_THRUST_  Fixed engine: its thrust. Rubber engine: the least thrust that
%   meets every thrust requirement at the concept's W/S (not the corner -
%   the wing is not free to move there).
    obj.geom.S_ref = S;
    if fixed
        obj.prop.T_SL = obj.prop.T_SL_fixed();
        return
    end
    for k = 1:3                                  % thrust -> engine size -> drag
        TWl = run_constraints(W0 / S, obj);
        need = max(TWl(~isnan(TWl)));
        if ~isfinite(need)
            error('concept_sizing:CannotFly', ['At W/S = %.1f psf a requirement needs more lift than ', ...
                'the wing has (CL > CLmax) - no thrust fixes that.'], W0 / S);
        end
        obj.prop.T_SL = W0 * need;
    end
end

function concept_growth_line_(ax, result, fixed)
%CONCEPT_GROWTH_LINE_  The path the fixed airplane takes as it gets heavier.
    if ~fixed || isnan(result.check.W_max), return, end
    W  = linspace(0.75 * result.W_TO, max(result.check.W_max, result.W_TO) * 1.05, 120);
    hold(ax, 'on');
    plot(ax, W / result.S_ref, result.T_SL ./ W, ':', 'Color', [0.45 0.1 0.6], 'LineWidth', 1.8, ...
         'DisplayName', 'weight-growth line (fixed S_ref and T_SL)');
    plot(ax, result.check.W_max / result.S_ref, result.T_SL / result.check.W_max, 'x', 'Color', [0.45 0.1 0.6], ...
         'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', sprintf('heaviest feasible W_TO %.0f lbf (%+.0f%%)', ...
         result.check.W_max, result.check.growth_pct));
end

function s = ternary_(c, a, b)
    if c, s = a; else, s = b; end
end
