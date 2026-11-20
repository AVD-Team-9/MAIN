function [T, fig] = concept_compare(json_files, varargin)
%CONCEPT_COMPARE  Analyse several fixed-geometry concepts and compare them side by side.
%
%   T = concept_compare({'concept2_rfp.json', 'concept3_rfp.json', 'concept5_rfp.json', 'concept6_rfp.json'})
%   T = concept_compare(files, 'save_png', true, 'title', "RFP thresholds")
%
%   Runs concept_sizing on every file (weight closed, geometry fixed) and
%   draws a grid: one tile per concept, each with ITS OWN requirement curves
%   (the concepts have different aerodynamics), its converged design point
%   (diamond) and its weight-growth line to the heaviest feasible weight (x).
%   All tiles share the same axes so they can be compared by eye.
%
%   OPTIONS
%     'labels'    string array (default "Concept N" from the file name)
%     'title'     overall title
%     'save_png'  false (default); true writes output/concept_comparison.png,
%                 or give a full path
%
%   OUTPUT  T  table, one row per concept: label, S_ref, W_TO, WS, TW,
%              W_fuel, OEW, all_pass, n_failed, worst requirement and its
%              margin, W_max (heaviest feasible W_TO), growth_pct, status

    concept_setup();
    p = inputParser;
    p.addParameter('labels', strings(0));
    p.addParameter('title', "");
    p.addParameter('save_png', false);
    p.parse(varargin{:});
    A = p.Results;
    files = string(json_files);
    n = numel(files);

    R = cell(1, n);
    rows = repmat(struct('label', "", 'S_ref', NaN, 'W_TO', NaN, 'WS', NaN, 'TW', NaN, 'W_fuel', NaN, ...
                         'OEW', NaN, 'all_pass', false, 'n_failed', NaN, 'worst', "", 'worst_margin_pct', NaN, ...
                         'W_max', NaN, 'growth_pct', NaN, 'status', "ok"), 1, n);
    for i = 1:n
        f = concept_setup(files(i));
        if numel(A.labels) >= i && strlength(A.labels(i)) > 0
            rows(i).label = A.labels(i);
        else
            tok = regexp(f, 'concept(\d+)', 'tokens', 'once');
            if ~isempty(tok), rows(i).label = "Concept " + tok{1};
            else, [~, b] = fileparts(f); rows(i).label = string(b); end
        end
        fprintf('  %s ...\n', rows(i).label);
        try
            [~, r] = evalc('concept_sizing(f, ''plot'', false, ''history'', false)');
            C = r.check;
            [wm, iw] = min(C.table.margin_pct);
            rows(i).S_ref = r.S_ref;  rows(i).W_TO = r.W_TO;  rows(i).WS = r.WS;  rows(i).TW = r.TW;
            rows(i).W_fuel = r.W_fuel;  rows(i).OEW = r.OEW;  rows(i).all_pass = C.all_pass;
            rows(i).n_failed = sum(~C.table.pass);  rows(i).worst = C.table.requirement(iw);
            rows(i).worst_margin_pct = wm;  rows(i).W_max = C.W_max;  rows(i).growth_pct = C.growth_pct;
            R{i} = r;
        catch ME
            rows(i).status = string(ME.message);
            fprintf('    FAILED: %s\n', ME.message);
        end
    end

    % ---- grid ------------------------------------------------------------------
    nc = min(2, n);  nr = ceil(n / nc);
    fig = figure('Name', 'Concept comparison (fixed geometry)', 'Color', 'w', 'Position', [60 60 1500 560 * nr]);
    tl  = tiledlayout(fig, nr, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    okk = find(cellfun(@(x) ~isempty(x), R));
    TWmax = min(1.2, max(0.6, 1.4 * max([rows(okk).TW])));
    first_ax = [];
    for i = 1:n
        ax = nexttile(tl);
        if isempty(R{i})
            text(ax, 0.5, 0.5, {char(rows(i).label), 'analysis failed:', char(rows(i).status)}, ...
                 'HorizontalAlignment', 'center', 'Interpreter', 'none');
            axis(ax, 'off');  continue
        end
        r = R{i};
        if rows(i).all_pass
            verdict = sprintf('all met, +%.0f%% growth', rows(i).growth_pct);
        else
            verdict = sprintf('%d FAILED (worst %s %+.0f%%)', rows(i).n_failed, rows(i).worst, rows(i).worst_margin_pct);
        end
        ttl = {sprintf('%s: S %.0f ft^2 fixed, W_{TO} %.0f lb, T/W %.3f', rows(i).label, r.S_ref, r.W_TO, r.TW), verdict};
        WS = wing_loading_sweep(r.obj, r.json);
        o = struct('axes', ax, 'legend', false, 'wall_labels', false, 'TW_max', TWmax, ...
                   'design_point', [r.WS, r.TW], 'title', {ttl});
        plot_constraint_diagram(r.obj, unique([WS, r.WS]), o);
        if r.obj.prop.is_fixed() && ~isnan(r.check.W_max)
            W = linspace(0.75 * r.W_TO, max(r.check.W_max, r.W_TO) * 1.05, 120);
            plot(ax, W / r.S_ref, r.T_SL ./ W, ':', 'Color', [0.45 0.1 0.6], 'LineWidth', 1.8, ...
                 'DisplayName', 'weight-growth line (fixed S_ref and T_SL)');
            plot(ax, r.check.W_max / r.S_ref, r.T_SL / r.check.W_max, 'x', 'Color', [0.45 0.1 0.6], ...
                 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', 'heaviest feasible W_TO');
        end
        if isempty(first_ax), first_ax = ax; end
    end
    if ~isempty(first_ax)
        lg = legend(first_ax, 'Interpreter', 'none');
        lg.Layout.Tile = 'east';
    end
    ttl = A.title;
    if strlength(ttl) == 0
        ttl = "Fixed-geometry concept analysis - each tile has its own requirement curves (diamond = converged design, x = heaviest feasible W_{TO})";
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
    disp(T(:, {'label', 'S_ref', 'W_TO', 'WS', 'TW', 'W_fuel', 'all_pass', 'n_failed', 'worst', 'worst_margin_pct', 'W_max', 'growth_pct'}));
end
