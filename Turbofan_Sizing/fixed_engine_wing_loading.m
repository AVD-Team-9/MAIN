function [WS, TW_avail, limit, ok] = fixed_engine_wing_loading(WS_sweep, TW_env, WS_wall, T_SL, W_TO, obj)
%FIXED_ENGINE_WING_LOADING  Wing loading to use when the engine is FIXED (a real engine).
%
%   [WS, TW_avail, limit, ok] = fixed_engine_wing_loading(WS_sweep, TW_env, WS_wall, T_SL, W_TO, obj)
%
%   Inputs
%     WS_sweep  wing-loading sweep [lbf/ft^2]
%     TW_env    thrust-loading envelope on that sweep (matching_envelope)
%     WS_wall   tightest wing-loading wall [lbf/ft^2]
%     T_SL      installed SLS thrust of the fixed engine(s) [lbf]
%     W_TO      takeoff weight [lbf]
%     obj       discipline bundle (only used to name the limiting wall)
%   Outputs
%     WS        wing loading to size the wing with [lbf/ft^2]
%     TW_avail  T/W the fixed engine gives at this weight, T_SL / W_TO
%     limit     what stops a smaller wing: the thrust requirement that the
%               envelope hits, or the wing-loading wall
%     ok        false if the engine cannot meet the requirements at ANY
%               wing loading at this weight (then WS is the least-thrust
%               corner, and limit says by how much the engine is short)
%
%   THE IDEA. With a rubber engine the sizing loop takes the least-thrust
%   corner of the diagram: the smallest engine that meets everything. With a
%   FIXED engine the thrust loading is no longer a choice - it is
%   T_SL / W_TO - so the design point slides along the horizontal line
%   T/W = TW_avail. Every wing loading where the envelope is BELOW that line
%   (and left of the wall) is feasible. The loop takes the LARGEST of them:
%   the SMALLEST wing that still meets every requirement, which gives the
%   lightest airplane (less wing weight and less wetted area). What stops
%   it is either a thrust requirement that rises with W/S (typically takeoff)
%   reaching the engine's T/W, or a wing-loading wall (landing, approach
%   speed, instantaneous g).
%
%   Other choices are legitimate - a bigger wing for fuel volume, field
%   performance margin or handling - but they are design decisions; this
%   function gives the minimum-wing answer and the caller can override it.

    TW_avail = T_SL / W_TO;
    feasible = (TW_env <= TW_avail) & (WS_sweep <= WS_wall);

    if any(feasible)
        ok = true;
        j  = find(feasible, 1, 'last');
        WS = WS_sweep(j);
        % What stops a smaller wing? The wall, if the next sweep point is
        % beyond it; otherwise the thrust requirement active at WS.
        if j == numel(WS_sweep) || WS_sweep(j + 1) > WS_wall
            [~, walls, names] = run_constraints(WS, obj);
            [~, iw] = min(walls);
            limit = "W/S wall: " + names(iw);
        else
            [TWl, ~, names] = run_constraints(WS_sweep(j + 1), obj);
            [~, it] = max(TWl);
            limit = names(it);
        end
    else
        ok = false;
        usable = TW_env;  usable(WS_sweep > WS_wall) = Inf;
        [TW_need, j] = min(usable);
        WS = WS_sweep(j);
        limit = sprintf('engine too small: needs T/W %.4f at %.1f psf, has %.4f', ...
                        TW_need, WS, TW_avail);
    end
end
