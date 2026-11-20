function [hist, converged, state] = sizing_mainloop(obj, opts)
%SIZING_MAINLOOP  Iterate sizing passes until weight and thrust stop moving.
%
%   [hist, converged, state] = sizing_mainloop(obj, opts)
%
%   Inputs
%     obj   discipline bundle. MUTATED: on return it holds the returned
%           airplane (T_SL and S_ref written).
%     opts  struct
%             .W0 .T0 .S0   starting guesses [lbf, lbf, ft^2]
%             .WS_sweep     wing-loading sweep
%             .tol          relative tolerance on BOTH W and T
%             .max_iter     pass limit
%             .relax        under-relaxation factor, 0 < w <= 1
%   Outputs
%     hist       1xN struct array, one sizing_pass row per pass
%     converged  true if both residuals fell below tol
%     state      struct .W0 .T0 .S0 (the returned airplane) and .n_iter
%
%   Identical to the TTPA sizing_mainloop with P (power) renamed T (thrust):
%
%     repeat
%        row = sizing_pass(obj, W0, T0, S0, WS_sweep)
%        stop if |W_new - W0|/W_new < tol AND |T_new - T0|/T_new < tol
%        otherwise relax all three states:  x <- x + w (x_new - x)
%
%   Both tests are needed: weight and thrust are coupled but distinct states.
%   Relaxation changes the PATH, never the answer - at the fixed point
%   x_new = x, so the relaxed step is x for any w.

    W0 = opts.W0;
    T0 = opts.T0;
    S0 = opts.S0;
    converged = false;
    hist = struct([]);

    for iter = 1:opts.max_iter
        % ---- one pass from the current states ----------------------------
        % A failure on the FIRST pass happens at the user's guesses, before
        % any relaxation has acted, so damping cannot fix it. It is re-thrown
        % with its own identifier so the driver can stop at once and say
        % "change the guesses" instead of retrying with smaller steps.
        try
            row = sizing_pass(obj, W0, T0, S0, opts.WS_sweep);
        catch ME
            if iter == 1
                error('sizing_mainloop:firstPassFailed', ...
                    ['The first pass failed at the starting guesses (W_0 = %.0f lbf, ', ...
                     'T_SL = %.0f lbf, S_ref = %.1f ft^2): %s  Change the guesses - ', ...
                     'relaxation cannot help a first pass. A wing guess near ', ...
                     'W_0 / (landing-wall W/S) is a good start.'], W0, T0, S0, ME.message);
            end
            rethrow(ME);
        end
        row.iter = iter;
        if isempty(hist)
            hist = row;
        else
            hist(iter) = row;
        end

        % ---- converged: take the pass's own answers, no relaxation --------
        if row.res_W < opts.tol && row.res_T < opts.tol
            W0 = row.W_TO_new;
            T0 = row.T_SL_new;
            S0 = row.S_ref_new;
            converged = true;
            break;
        end

        % ---- otherwise relax all three states ------------------------------
        W0 = SizingSteps.relax(W0, row.W_TO_new,  opts.relax);
        T0 = SizingSteps.relax(T0, row.T_SL_new,  opts.relax);
        S0 = SizingSteps.relax(S0, row.S_ref_new, opts.relax);
    end

    % ---- leave the bundle holding the airplane returned ------------------------
    obj.prop.T_SL  = T0;
    obj.geom.S_ref = S0;
    state = struct('W0', W0, 'T0', T0, 'S0', S0, 'n_iter', iter);

    if ~converged
        warning('sizing_mainloop:notConverged', ...
            ['The loop did not close in %d passes at relaxation %.2f. Last W_TO = ', ...
             '%.1f lbf, residuals %.2e (weight), %.2e (thrust).'], ...
            opts.max_iter, opts.relax, W0, hist(end).res_W, hist(end).res_T);
    end
end
