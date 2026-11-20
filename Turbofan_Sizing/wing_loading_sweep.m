function [WS_sweep, WS_wall] = wing_loading_sweep(obj, json_path)
%WING_LOADING_SWEEP  The wing-loading sweep the constraint analysis is solved on.
%
%   [WS_sweep, WS_wall] = wing_loading_sweep(obj)
%   [WS_sweep, WS_wall] = wing_loading_sweep(obj, json_path)
%
%   Same as the TTPA version: constraints.wing_loading_range_psf sampled at
%   no fewer than 400 points, plus the landing wall appended at its exact
%   location (just inside it), so a least-thrust corner that sits on the
%   wall is not quantised to the grid spacing.
%
%   obj must already hold S_ref and T_SL, because run_constraints evaluates
%   every condition (the geometry-dependent drag needs a wing, and the engine
%   size regressions need a thrust).

    if nargin < 2
        json_path = tf_requirements_path();
    end
    J  = jsondecode(fileread(json_path));
    Cn = J.constraints;

    WS_sweep = linspace(Cn.wing_loading_range_psf(1), Cn.wing_loading_range_psf(2), ...
                        max(Cn.wing_loading_points, 400));

    [~, walls] = run_constraints(mean(WS_sweep), obj);
    WS_wall = min(walls(~isnan(walls)));

    if isempty(WS_wall) || ~isfinite(WS_wall)
        WS_wall = Inf;
    elseif WS_wall > WS_sweep(1) && WS_wall < WS_sweep(end)
        WS_sweep = unique([WS_sweep, WS_wall * (1 - 1e-9)]);
    end
end
