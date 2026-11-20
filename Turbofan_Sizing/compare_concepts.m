function [T, fig] = compare_concepts(json_files, varargin)
%COMPARE_CONCEPTS  A grid of constraint diagrams: one tile per design concept, each with ITS OWN curves.
%
%   T = compare_concepts({'prm2_concept2_rfp_requirements.json', 'prm2_concept3_rfp_requirements.json', ...
%                         'prm2_concept5_rfp_requirements.json', 'prm2_concept6_rfp_requirements.json'}, ...
%                        'save_png', true)
%   T = compare_concepts(files, 'mode', "fixed_weight", 'W_TO', 14000)
%
%   WHY A GRID AND NOT ONE DIAGRAM. compare_engine_points overlays several
%   ENGINES on one airplane's curves, which is fine because the aerodynamics
%   are the same. Different CONCEPTS have different aerodynamics (CD0, Oswald
%   e, CLmax, sweep, drag-divergence Mach), so each one has its own
%   requirement curves; drawing them all over one concept's curves would
%   mislead. Each tile here is solved on its own airplane, and all tiles
%   share the same axes so they can be compared by eye.
%
%   MODES
%     "sized" (default)   run_tf_sizing on every file: each tile shows that
%                         concept's CONVERGED airplane and its design point.
%     "fixed_weight"      run_tf_constraints at one W_TO ('W_TO', default
%                         each file's W_TO_estimate_lbf): fast, no sizing.
%
%   OPTIONS
%     'labels'    string array (default "Concept N" from the file name, or
%                 the file name)
%     'title'     overall title
%     'save_png'  false (default); true writes output/concept_comparison.png,
%                 or give a full path
%
%   OUTPUT  T  table, one row per concept: label, W_TO, S_ref, WS, TW,
%              TW_corner, margin_pct, CD0, e, CLmax_clean, W_fuel, governing
%              mission, wing_set_by, status

    p = inputParser;
    p.addParameter('mode', "sized", @(x) ismember(string(x), ["sized" "fixed_weight"]));
    p.addParameter('W_TO', []);
    p.addParameter('labels', strings(0));
    p.addParameter('title', "");
    p.addParameter('save_png', false);
    p.parse(varargin{:});
    A = p.Results;
    files = string(json_files);
    n = numel(files);
    mode = string(A.mode);

    ws0 = warning('off', 'TfProp:ThrustOutOfRange');
    restore = onCleanup(@() warning(ws0));

    R = cell(1, n);
    rows = repmat(struct('label', "", 'W_TO', NaN, 'S_ref', NaN, 'WS', NaN, 'TW', NaN, 'TW_corner', NaN, ...
                         'margin_pct', NaN, 'CD0', NaN, 'e', NaN, 'CLmax_clean', NaN, 'W_fuel', NaN, ...
                         'governing', "", 'wing_set_by', "", 'status', "ok"), 1, n);
    for i = 1:n
        f = files(i);
        if numel(A.labels) >= i && strlength(A.labels(i)) > 0
            rows(i).label = A.labels(i);
        else
            tok = regexp(f, 'concept(\d+)', 'tokens', 'once');
            if ~isempty(tok), rows(i).label = "Concept " + tok{1};
            else, [~, b] = fileparts(f); rows(i).label = string(b); end
        end
        fprintf('  %s (%s) ...\n', rows(i).label, f);
        try
            if mode == "sized"
                [~, r] = evalc('run_tf_sizing(''json'', f, ''plot'', false, ''history'', false)');
                o = r.obj;
                rows(i).W_TO = r.W_TO;  rows(i).S_ref = r.S_ref;
                rows(i).WS = r.W_TO / r.S_ref;  rows(i).TW = r.T_SL / r.W_TO;
                rows(i).W_fuel = r.W_fuel;  rows(i).governing = string(r.governing_mission);
                rows(i).wing_set_by = string(r.driving);
            else
                W = A.W_TO;
                if isempty(W), J = jsondecode(fileread(f)); W = J.constraints.W_TO_estimate_lbf; end
                [~, q] = evalc('run_tf_constraints(''json'', f, ''plot'', false, ''W_TO'', W)');
                o = q.obj;
                rows(i).W_TO = W;  rows(i).S_ref = o.geom.S_ref;
                rows(i).WS = W / o.geom.S_ref;  rows(i).TW = o.prop.T_SL / W;
                rows(i).wing_set_by = "fixed-weight constraint analysis";
            end
            WS = wing_loading_sweep(o, f);
            [~, ~, ~, ~, TWc] = matching_envelope(WS, o);
            rows(i).TW_corner  = TWc;
            rows(i).margin_pct = 100 * (rows(i).TW / TWc - 1);
            rows(i).CD0 = o.aero.CD0;  rows(i).e = o.aero.e;  rows(i).CLmax_clean = o.aero.CLmax_clean;
            R{i} = struct('obj', o, 'WS', WS, 'file', f);
        catch ME
            rows(i).status = string(ME.message);
            fprintf('    FAILED: %s\n', ME.message);
        end
    end

    % ---- the grid ---------------------------------------------------------------
    ok = find(cellfun(@(x) ~isempty(x), R));
    nc = min(2, n);  nr = ceil(n / nc);
    fig = figure('Name', 'Concept comparison', 'Color', 'w', 'Position', [60 60 1500 560 * nr]);
    tl  = tiledlayout(fig, nr, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    TWmax = min(1.2, max(0.6, 1.25 * max([rows(ok).TW, 1.8 * [rows(ok).TW_corner]])));
    first_ax = [];
    for i = 1:n
        ax = nexttile(tl);
        if isempty(R{i})
            text(ax, 0.5, 0.5, {char(rows(i).label), 'did not close:', char(extractBefore(rows(i).status + "  Change", "  Change"))}, ...
                 'HorizontalAlignment', 'center', 'Interpreter', 'none');
            axis(ax, 'off');
            continue
        end
        if mode == "sized"
            ttl = sprintf('%s: W_{TO} %.0f lb, S %.0f ft^2, T/W %.3f (%+.0f%% over corner)', rows(i).label, ...
                          rows(i).W_TO, rows(i).S_ref, rows(i).TW, rows(i).margin_pct);
        else
            ttl = sprintf('%s at W_{TO} %.0f lb (corner T/W %.3f)', rows(i).label, rows(i).W_TO, rows(i).TW_corner);
        end
        o = struct('axes', ax, 'legend', false, 'wall_labels', false, 'TW_max', TWmax, ...
                   'design_point', [rows(i).WS, rows(i).TW], 'title', ttl);
        plot_constraint_diagram(R{i}.obj, R{i}.WS, o);
        if isempty(first_ax), first_ax = ax; end
    end
    if ~isempty(first_ax)
        % One shared legend: every tile has the same requirement list. The
        % corner and design-point entries carry the FIRST tile's numbers -
        % read each tile's own numbers from its title.
        lg = legend(first_ax, 'Interpreter', 'none');
        lg.Layout.Tile = 'east';
    end
    ttl = A.title;
    if strlength(ttl) == 0
        ttl = "Concept comparison - each tile has its own requirement curves";
        if mode == "sized", ttl = ttl + " (converged designs, diamond = design point)"; end
    end
    title(tl, ttl, 'FontWeight', 'bold');

    if (islogical(A.save_png) && A.save_png) || (~islogical(A.save_png) && strlength(string(A.save_png)) > 0)
        if islogical(A.save_png)
            out = fullfile(fileparts(mfilename('fullpath')), 'output', 'concept_comparison.png');
        else
            out = string(A.save_png);
        end
        if ~exist(fileparts(out), 'dir'), mkdir(fileparts(out)); end
        exportgraphics(fig, out, 'Resolution', 170);
        fprintf('  wrote %s\n', out);
    end

    T = struct2table(rows);
    disp(T(:, {'label', 'W_TO', 'S_ref', 'WS', 'TW', 'TW_corner', 'margin_pct', 'CD0', 'e', 'CLmax_clean', 'W_fuel', 'wing_set_by'}));
end
