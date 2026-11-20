function [P, fig] = compare_engine_points(json_files, varargin)
%COMPARE_ENGINE_POINTS  One constraint diagram with the design point of several engines.
%
%   P = compare_engine_points(["prm2_f124_requirements.json", "prm2_f404_requirements.json"])
%   P = compare_engine_points(files, 'mode', "sized", 'save_png', true)
%   P = compare_engine_points(files, 'mode', "common_weight", 'W_TO', 14000)
%
%   Each file is the SAME airplane and requirements with a different engine
%   (for example prm2_f124_requirements.json and prm2_f404_requirements.json).
%   The function puts every engine's design point on ONE diagram, so the
%   engine trade (RFP-4-P05-R01.1) can be seen at a glance.
%
%   MODES
%     "sized" (default)  run_tf_sizing on every file; each point is that
%                        engine's CONVERGED airplane: (W/S, T/W) =
%                        (W_TO/S_ref, T_SL/W_TO). This is the honest
%                        comparison - each engine gets the airplane it needs.
%                        Slow (one sizing run per file).
%     "common_weight"    no sizing: every engine at the same W_TO ('W_TO',
%                        default the first file's W_TO_estimate_lbf). Each
%                        point is T/W = T_SL_engine / W_TO at the smallest
%                        wing that engine allows (fixed_engine_wing_loading),
%                        so you see how much thrust each engine brings to
%                        the same airplane. Fast.
%
%   THE CURVES are solved on the FIRST file's airplane (its converged one in
%   "sized" mode). T/W and W/S are normalised, so the requirement curves are
%   nearly the same for every engine, but not exactly: drag depends on the
%   wetted-area ratio (wing size) and the curves are drawn for one
%   airplane. Read each point against the curves as "approximately", and use
%   each engine's own run_tf_sizing diagram for its exact margins.
%
%   OPTIONS (name/value)
%     'mode'      "sized" | "common_weight"
%     'W_TO'      common weight [lbf] for "common_weight"
%     'labels'    string array, one per file (default: each fixed engine's
%                 name, or the file name)
%     'title'     figure title
%     'save_png'  false (default); true writes output/engine_comparison_diagram.png,
%                 or give a full path
%
%   OUTPUT  P  table: label, W_TO, S_ref, WS, TW, T_uninstalled, W_fuel
%              (NaN in "common_weight" mode), limit (what sets the wing)

    p = inputParser;
    p.addParameter('mode', "sized", @(x) ismember(string(x), ["sized" "common_weight"]));
    p.addParameter('W_TO', []);
    p.addParameter('labels', strings(0));
    p.addParameter('title', "");
    p.addParameter('save_png', false);
    p.parse(varargin{:});
    A = p.Results;
    files = string(json_files);
    n = numel(files);
    mode = string(A.mode);

    rows = struct('label', strings(1, n), 'W_TO', NaN(1, n), 'S_ref', NaN(1, n), ...
                  'WS', NaN(1, n), 'TW', NaN(1, n), 'T_uninstalled', NaN(1, n), ...
                  'W_fuel', NaN(1, n), 'limit', strings(1, n));
    objs = cell(1, n);
    ws0 = warning('off', 'TfProp:ThrustOutOfRange');
    restore = onCleanup(@() warning(ws0));

    for i = 1:n
        f = files(i);
        if mode == "sized"
            fprintf('  sizing %s ...\n', f);
            [~, r] = evalc('run_tf_sizing(''json'', f, ''plot'', false, ''history'', false)');
            o = r.obj;
            rows.W_TO(i)  = r.W_TO;   rows.S_ref(i) = r.S_ref;
            rows.WS(i)    = r.W_TO / r.S_ref;
            rows.TW(i)    = r.T_SL / r.W_TO;
            rows.T_uninstalled(i) = r.engine.T_uninstalled_per_engine;
            rows.W_fuel(i) = r.W_fuel;
            rows.limit(i)  = string(r.driving);
        else
            if isempty(A.W_TO)
                J = jsondecode(fileread(files(1)));
                A.W_TO = J.constraints.W_TO_estimate_lbf;
            end
            [~, R] = evalc('run_tf_constraints(''json'', f, ''plot'', false, ''W_TO'', A.W_TO)');
            o = R.obj;              % the points are placed below, on one sweep
        end
        objs{i} = o;
        if numel(A.labels) >= i && strlength(A.labels(i)) > 0
            rows.label(i) = A.labels(i);
        elseif o.prop.is_fixed()
            rows.label(i) = string(o.prop.fixed_engine.name);
        else
            [~, b] = fileparts(f);  rows.label(i) = string(b);
        end
    end

    % ---- curves on the FIRST airplane -----------------------------------------
    ref = objs{1};
    if mode == "common_weight"
        % Put the reference airplane at the common weight with its own
        % smallest wing, then place every engine on that airplane.
        WS = wing_loading_sweep(ref, files(1));
        for i = 1:n
            o = objs{i};
            [TW_env, WS_wall] = matching_envelope(WS, o);
            T = o.prop.T_SL;
            if o.prop.is_fixed(), T = o.prop.T_SL_fixed(); end
            [ws_i, tw_i, lim_i] = fixed_engine_wing_loading(WS, TW_env, WS_wall, T, A.W_TO, o);
            rows.W_TO(i) = A.W_TO;  rows.WS(i) = ws_i;  rows.TW(i) = tw_i;
            rows.S_ref(i) = A.W_TO / ws_i;  rows.limit(i) = string(lim_i);
            rows.T_uninstalled(i) = T / o.prop.n_engines / (1 - o.prop.installation_loss);
        end
        ref.geom.S_ref = rows.S_ref(1);
    end
    WS = wing_loading_sweep(ref, files(1));

    ttl = A.title;
    if strlength(ttl) == 0
        % Two lines (a cell array): one long line is cut off by the legend.
        if mode == "sized"
            ttl = {'Engine comparison: each point is its own converged airplane', ...
                   sprintf('curves drawn for the %s airplane, W_{TO} %.0f lbf', rows.label(1), rows.W_TO(1))};
        else
            ttl = {sprintf('Engine comparison at a common W_{TO} = %.0f lbf', A.W_TO), ...
                   sprintf('curves drawn for the %s airplane', rows.label(1))};
        end
    end
    o = struct('title', ttl, 'TW_max', max(0.6, 1.15 * max(rows.TW)));
    [fig, ~] = plot_constraint_diagram(ref, WS, o);
    ax = fig.CurrentAxes;

    % ---- the engine points ------------------------------------------------------
    markers = {'p', 's', '^', 'v', 'h', 'd'};
    colors  = [0.85 0.10 0.10; 0.10 0.35 0.85; 0.10 0.60 0.20; 0.60 0.20 0.70; 0.90 0.50 0.00; 0.30 0.30 0.30];
    for i = 1:n
        c = colors(mod(i-1, size(colors,1)) + 1, :);
        if mode == "sized"
            lbl = sprintf('%s: %.0f lb, %.0f ft^2 (%.1f psf, T/W %.3f)', rows.label(i), rows.W_TO(i), ...
                          rows.S_ref(i), rows.WS(i), rows.TW(i));
        else
            lbl = sprintf('%s: %.1f psf, T/W %.3f', rows.label(i), rows.WS(i), rows.TW(i));
        end
        plot(ax, rows.WS(i), rows.TW(i), markers{mod(i-1, numel(markers)) + 1}, 'MarkerSize', 14, ...
             'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'LineWidth', 1.2, 'DisplayName', lbl);
        text(ax, rows.WS(i) + 1.2, rows.TW(i) + 0.012, rows.label(i), 'FontWeight', 'bold', 'Color', c);
    end

    if (islogical(A.save_png) && A.save_png) || (~islogical(A.save_png) && strlength(string(A.save_png)) > 0)
        if islogical(A.save_png)
            out = fullfile(fileparts(mfilename('fullpath')), 'output', 'engine_comparison_diagram.png');
        else
            out = string(A.save_png);
        end
        if ~exist(fileparts(out), 'dir'), mkdir(fileparts(out)); end
        exportgraphics(fig, out, 'Resolution', 200);
        fprintf('  wrote %s\n', out);
    end

    P = struct2table(structfun(@(v) v(:), rows, 'UniformOutput', false));
    disp(P);
end
