function [fig, D] = plot_constraint_diagram(obj, WS, opts)
%PLOT_CONSTRAINT_DIAGRAM  Draw the T/W - W/S constraint (matching) diagram.
%
%   [fig, D] = plot_constraint_diagram(obj, WS)
%   [fig, D] = plot_constraint_diagram(obj, WS, opts)
%
%   Inputs
%     obj   discipline bundle; must hold S_ref and T_SL (the geometry-
%           dependent drag and the engine-size models need them)
%     WS    wing-loading sweep [lbf/ft^2]
%     opts  optional struct
%             .design_point  [W/S, T/W] of YOUR chosen point, drawn as a
%                            filled diamond (default: none)
%             .title         figure title (default below)
%             .TW_max        top of the vertical axis (default: automatic)
%             .save_png      full path of a PNG to write (default: none)
%             .axes          draw into this axes instead of a new figure
%             .legend        false hides the legend (default true)
%             .wall_labels   false drops the text on the W/S walls (default true)
%   Outputs
%     fig   figure handle
%     D     struct with everything that was drawn: .WS .TW_limits .names
%           .WS_walls .TW_env .WS_max .WS_best .TW_best .driving_at_best
%
%   NEW - the TTPA code computed the matching envelope but never drew it.
%
%   HOW TO READ IT
%     * Each coloured curve is one requirement: the airplane needs at least
%       that much T/W at that W/S.
%     * The dashed vertical line is the landing wall: W/S must be at or LEFT
%       of it.
%     * The shaded region (above every curve, left of the wall) is every
%       airplane that meets every requirement.
%     * The open circle is the least-thrust corner of that region - the
%       smallest engine the requirements permit.
%     * Multiply a T/W read off this diagram by W_TO to get the installed
%       sea-level-static thrust of ALL engines.

    if nargin < 3, opts = struct(); end

    WS = reshape(WS, 1, []);
    [TW_limits, WS_walls, names] = run_constraints(WS, obj);
    [TW_env, WS_max, driving, WS_best, TW_best] = matching_envelope(WS, obj);

    % ---- vertical-axis range ------------------------------------------------
    % Default: about twice the least-thrust T/W, kept between 0.6 and 1.2, so
    % the interesting corner fills the plot and the steep ends of the bucket
    % curves run off the top instead of squashing it.
    if isfield(opts, 'TW_max')
        TW_top = opts.TW_max;
    elseif ~isnan(TW_best)
        TW_top = min(1.2, max(0.6, 1.8 * TW_best));
    else
        TW_top = 1.0;
    end

    % opts.axes: draw into an existing axes (e.g. one tile of a grid of
    % diagrams, compare_concepts) instead of a new figure.
    if isfield(opts, 'axes') && ~isempty(opts.axes)
        ax  = opts.axes;
        fig = ancestor(ax, 'figure');
    else
        fig = figure('Name', 'Constraint diagram', 'Color', 'w', 'Position', [100 100 900 620]);
        ax  = axes(fig);
    end
    hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');

    % ---- 1. shade the feasible region --------------------------------------
    % Above the envelope, left of the wall, below the top of the axes.
    ok = isfinite(TW_env) & WS <= WS_max & TW_env <= TW_top;
    if any(ok)
        xs = WS(ok);  ys = TW_env(ok);
        patch(ax, [xs, fliplr(xs)], [ys, TW_top * ones(size(ys))], ...
              [0.80 0.92 0.80], 'EdgeColor', 'none', 'FaceAlpha', 0.6, ...
              'DisplayName', 'feasible region');
    end

    % ---- 2. one curve per thrust-loading requirement -----------------------
    % MATLAB's lines() palette has only 7 distinct colours and then repeats,
    % so after the 7th curve the line style changes too (solid -> dash-dot
    % -> dotted) to keep every curve distinguishable.
    is_thrust = find(~all(isnan(TW_limits), 2));
    cmap   = lines(7);
    styles = {'-', '-.', ':'};
    for i = 1:numel(is_thrust)
        k = is_thrust(i);
        y = TW_limits(k, :);
        y(~isfinite(y)) = NaN;          % Inf (CL > CLmax) -> gap in the line
        c = cmap(mod(i-1, 7) + 1, :);
        s = styles{mod(floor((i-1)/7), numel(styles)) + 1};
        plot(ax, WS, y, s, 'Color', c, 'LineWidth', 1.6, ...
             'DisplayName', char(names(k)));
    end

    % ---- 3. wing-loading walls ----------------------------------------------
    wall_txt = ~isfield(opts, 'wall_labels') || opts.wall_labels;
    for k = find(~isnan(WS_walls))
        lbl = '';  if wall_txt, lbl = char(names(k)); end
        xline(ax, WS_walls(k), '--', lbl, 'LineWidth', 1.6, ...
              'Color', [0.2 0.2 0.2], 'LabelVerticalAlignment', 'bottom', ...
              'DisplayName', char(names(k)));
    end

    % ---- 4. the envelope, the least-thrust corner, and the design point -----
    plot(ax, WS(WS <= WS_max), TW_env(WS <= WS_max), 'k-', 'LineWidth', 2.4, ...
         'DisplayName', 'envelope (max of requirements)');

    if ~isnan(WS_best)
        plot(ax, WS_best, TW_best, 'ko', 'MarkerSize', 10, 'LineWidth', 2, ...
             'DisplayName', sprintf('least-thrust point (%.1f psf, %.3f)', WS_best, TW_best));
    end

    if isfield(opts, 'design_point') && ~isempty(opts.design_point)
        dp = opts.design_point;
        plot(ax, dp(1), dp(2), 'kd', 'MarkerFaceColor', [0.95 0.75 0.2], ...
             'MarkerSize', 10, 'DisplayName', sprintf('design point (%.1f psf, %.3f)', dp(1), dp(2)));
    end

    % ---- labels -------------------------------------------------------------
    xlim(ax, [WS(1), WS(end)]);
    ylim(ax, [0, TW_top]);
    xlabel(ax, 'Wing loading  (W/S)_{TO}  [lbf/ft^2]');
    ylabel(ax, 'Thrust loading  (T/W)_{TO}  [-]   (installed, sea-level static)');
    if isfield(opts, 'title')
        title(ax, opts.title);
    else
        title(ax, 'Constraint diagram - feasible region is ABOVE the curves and LEFT of the wall');
    end
    % opts.legend = false hides the legend (grids share one); opts.wall_labels
    % = false drops the text on the wing-loading walls (small tiles).
    if ~isfield(opts, 'legend') || opts.legend
        legend(ax, 'Location', 'northeastoutside', 'Interpreter', 'none');
    end

    % ---- optional PNG ---------------------------------------------------------
    if isfield(opts, 'save_png') && strlength(string(opts.save_png)) > 0
        out_dir = fileparts(opts.save_png);
        if ~isempty(out_dir) && ~exist(out_dir, 'dir'), mkdir(out_dir); end
        exportgraphics(fig, opts.save_png, 'Resolution', 200);
    end

    % ---- everything that was drawn --------------------------------------------
    D = struct('WS', WS, 'TW_limits', TW_limits, 'names', names, ...
               'WS_walls', WS_walls, 'TW_env', TW_env, 'WS_max', WS_max, ...
               'WS_best', WS_best, 'TW_best', TW_best, 'driving', driving);
    if ~isnan(WS_best)
        D.driving_at_best = driving(find(WS == WS_best, 1));
    else
        D.driving_at_best = "";
    end
end
