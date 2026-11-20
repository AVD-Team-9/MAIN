function C = concept_constraint_check(obj, W_TO)
%CONCEPT_CONSTRAINT_CHECK  Every requirement checked on a FIXED airplane at a given weight.
%
%   C = concept_constraint_check(obj, W_TO)
%
%   Inputs
%     obj    discipline bundle with the FIXED wing (obj.geom.S_ref) and the
%            engine (obj.prop.T_SL) already set
%     W_TO   takeoff weight to check [lbf]
%   Output struct C
%     .WS, .TW          design point: W_TO/S_ref and T_SL/W_TO
%     .table            one row per requirement: name, type, required (T/W
%                       needed, or the W/S wall), available, margin_pct, pass
%     .all_pass         true if every requirement is met
%     .worst            name of the requirement with the smallest margin
%     .W_max            HEAVIEST takeoff weight at which every requirement is
%                       still met with this wing and engine [lbf] (NaN if none
%                       in the search range)
%     .W_max_limit      the requirement that stops a heavier airplane
%     .growth_pct       100 (W_max / W_TO - 1): weight-growth margin
%
%   HOW IT DIFFERS FROM THE SIZING CODE. Turbofan_Sizing CHOOSES the wing
%   (and, for a rubber engine, the thrust) so the design point sits on the
%   edge of the feasible region. Here the wing and the engine are given, so
%   the design point is wherever the weight puts it,
%       W/S = W_TO / S_ref,      T/W = T_SL / W_TO,
%   and each requirement is a pass/fail CHECK with a margin:
%       thrust requirement:   margin = (T/W available) / (T/W needed) - 1
%       wing-loading wall:    margin = (W/S wall) / (W/S) - 1
%
%   WEIGHT-GROWTH MARGIN. With S_ref and T_SL fixed, a heavier airplane moves
%   along the curve T/W = T_SL / (S_ref W/S) - right and down on the diagram -
%   until it meets the first requirement: a rising thrust curve (takeoff,
%   turns) or a wall (landing, instantaneous g). That weight is W_max. It
%   says how much the concept can grow (equipment, fuel, a heavier structure
%   estimate) before it stops meeting the requirements - the usual way to
%   express a fixed concept's margin.

    S = obj.geom.S_ref;
    T = obj.prop.T_SL;
    C.WS = W_TO / S;
    C.TW = T / W_TO;

    [TWl, walls, names, types] = run_constraints(C.WS, obj);
    n = numel(names);
    req = NaN(n, 1);  avail = NaN(n, 1);  marg = NaN(n, 1);
    for k = 1:n
        if isnan(walls(k))                       % thrust-loading requirement
            req(k) = TWl(k);  avail(k) = C.TW;
            marg(k) = 100 * (C.TW / TWl(k) - 1); % Inf need -> -100 %
        else                                     % wing-loading wall
            req(k) = walls(k);  avail(k) = C.WS;
            marg(k) = 100 * (walls(k) / C.WS - 1);
        end
    end
    C.table = table(names(:), types(:), req, avail, marg, marg >= 0, ...
        'VariableNames', {'requirement', 'type', 'required', 'available', 'margin_pct', 'pass'});
    C.all_pass = all(C.table.pass);
    [~, iw] = min(marg);
    C.worst = names(iw);

    % ---- heaviest feasible weight on the fixed-airplane growth line ----------
    feas = @(W) feasible_(obj, W, S, T);
    Wv = W_TO * linspace(0.4, 3.0, 261);
    ok = arrayfun(feas, Wv);
    C.W_max = NaN;  C.W_max_limit = "";  C.growth_pct = NaN;
    if any(ok)
        j = find(ok, 1, 'last');
        if j < numel(Wv)                         % refine the edge by bisection
            lo = Wv(j);  hi = Wv(j + 1);
            for it = 1:40
                mid = 0.5 * (lo + hi);
                if feas(mid), lo = mid; else, hi = mid; end
            end
            C.W_max = lo;
            [~, C.W_max_limit] = feasible_(obj, hi, S, T);
        else
            C.W_max = Wv(end);
            C.W_max_limit = "beyond the search range (3 x W_TO)";
        end
        C.growth_pct = 100 * (C.W_max / W_TO - 1);
    end
end

%% ---------------------------------------------------------------------------
function [ok, why] = feasible_(obj, W, S, T)
%FEASIBLE_  Does the fixed airplane meet every requirement at weight W?
    WS = W / S;
    [TWl, walls, names] = run_constraints(WS, obj);
    m = inf(numel(names), 1);
    th = isnan(walls);
    m(th)  = (T / W) ./ TWl(th) - 1;
    m(~th) = walls(~th) / WS - 1;
    ok = all(m >= 0);
    [~, i] = min(m);
    why = names(i);
end
