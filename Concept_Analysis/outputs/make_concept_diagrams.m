% Constraint diagrams per concept: RFP and team requirements, fixed (VSP) wing
% and rubber (resized) wing. Calls the existing drivers only - no code changes.
root = 'C:\Users\ganon\Downloads\IHW3_Code';
CA = fullfile(root, 'Concept_Analysis');  TS = fullfile(root, 'Turbofan_Sizing');
set(0, 'DefaultFigureVisible', 'off');
sets = struct('tag', {'rfp', 'team'}, 'ca', {'rfp', 'team'}, 'ts', {'rfp_requirements', 'requirements'}, ...
              'title', {'RFP thresholds', 'Team requirements'});
for n = [2 3 5 6]
    od = fullfile(CA, 'output', sprintf('concept%d', n));
    if ~exist(od, 'dir'), mkdir(od); end
    lg = strings(0);
    for s = sets
        % ---------------- fixed wing (Concept_Analysis) ----------------
        cd(CA);
        f1 = sprintf('concept%d_%s.json', n, s.ca);
        png1 = fullfile(od, sprintf('concept%d_%s_fixed_wing.png', n, s.tag));
        [~, rc] = evalc('concept_sizing(f1, ''history'', false, ''plot'', true, ''save_png'', png1)');
        close all
        Cn = rc.check.table;  fails = Cn.requirement(~Cn.pass);
        lg(end+1) = sprintf('%s, FIXED wing (inputs/%s): W_TO %.0f lb, S %.1f ft^2 (fixed), W/S %.1f, T/W %.3f -> %s', ...
            s.title, f1, rc.W_TO, rc.S_ref, rc.WS, rc.TW, ternary(isempty(fails), 'all met', "FAILS " + strjoin(fails, ', ')));
        fprintf('%s\n', lg(end));

        % ---------------- rubber wing (Turbofan_Sizing) ----------------
        cd(TS);
        f2 = sprintf('prm2_concept%d_%s.json', n, s.ts);
        try
            [~, r] = evalc('run_tf_sizing(''json'', f2, ''plot'', false, ''history'', false)');
            o = r.obj;
            WS = wing_loading_sweep(o, f2);
            png2 = fullfile(od, sprintf('concept%d_%s_rubber_wing.png', n, s.tag));
            opts = struct('design_point', [r.W_TO / r.S_ref, r.T_SL / r.W_TO], 'save_png', png2, ...
                'title', sprintf('Concept %d %s, rubber wing (F124): W_{TO} %.0f lbf, S %.1f ft^2', ...
                                 n, s.ca, r.W_TO, r.S_ref));
            plot_constraint_diagram(o, WS, opts);  close all
            lg(end+1) = sprintf('%s, RUBBER wing (Turbofan_Sizing/%s): converged W_TO %.0f lb, S %.1f ft^2, W/S %.1f, T/W %.3f', ...
                s.title, f2, r.W_TO, r.S_ref, r.W_TO / r.S_ref, r.T_SL / r.W_TO);
        catch e
            % Did not close: draw the requirements at the fixed-wing W_TO, the
            % wing at the least-thrust corner, and the F124 point at that weight.
            W = rc.W_TO;
            o = tf_disciplines(f2);
            o.prop.T_SL = o.prop.T_SL_fixed();
            o.geom.S_ref = rc.S_ref;
            ws0 = warning('off', 'all');
            for it = 1:20
                WS = wing_loading_sweep(o, f2);
                [~, ~, ~, WS_best] = matching_envelope(WS, o);
                if isnan(WS_best), break, end
                Snew = W / WS_best;
                if abs(Snew - o.geom.S_ref) / Snew < 1e-6, break, end
                o.geom.S_ref = Snew;
            end
            warning(ws0);
            png2 = fullfile(od, sprintf('concept%d_%s_rubber_wing_NOT_CONVERGED.png', n, s.tag));
            opts = struct('design_point', [W / o.geom.S_ref, o.prop.T_SL / W], 'save_png', png2, ...
                'title', sprintf('Concept %d %s, rubber wing: NOT CONVERGED (F124 at %.0f lbf, %.0f ft^2)', ...
                                 n, s.ca, W, o.geom.S_ref));
            plot_constraint_diagram(o, WS, opts);  close all
            m0 = char(e.message);
            lg(end+1) = sprintf('%s, RUBBER wing (Turbofan_Sizing/%s): DID NOT CONVERGE (%s). Diagram drawn at the fixed-wing W_TO %.0f lb with the wing at the least-thrust W/S (%.1f ft^2); the diamond is the F124 there.', ...
                s.title, f2, m0(1:min(end, 150)), W, o.geom.S_ref);
        end
        fprintf('%s\n', lg(end));
    end
    fid = fopen(fullfile(od, 'README.txt'), 'w');
    fprintf(fid, 'Concept %d constraint diagrams (F124-GA-100, both RFP missions, current inputs and code).\n', n);
    fprintf(fid, 'FIXED wing  = Concept_Analysis/concept_sizing: VSP wing area held, W_TO closed.\n');
    fprintf(fid, 'RUBBER wing = Turbofan_Sizing/run_tf_sizing: wing area chosen by the loop, W_TO closed.\n');
    fprintf(fid, 'Diamond = the design point; open circle = least-thrust corner of the feasible region.\n\n');
    fprintf(fid, '%s\n\n', lg);
    fclose(fid);
end
set(0, 'DefaultFigureVisible', 'on');

function out = ternary(c, a, b)
    if c, out = a; else, out = b; end
end
