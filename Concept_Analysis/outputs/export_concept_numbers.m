% Numerical exports per concept: RFP and team requirements, fixed (VSP) wing
% and rubber (resized) wing. Same cases as make_concept_diagrams.m; calls the
% existing drivers only - no code changes. Writes into output/conceptN/:
%   conceptN_<set>_<wing>_output.txt        full console report of the run
%   conceptN_<set>_<wing>_requirements.csv  every requirement: needed, available, margin, pass
%   conceptN_<set>_<wing>_weights.csv       weight build-up, payload, fuel, W_TO
%   conceptN_summary.csv                    one row per case (4 rows)
root = 'C:\Users\ganon\Downloads\IHW3_Code';
CA = fullfile(root, 'Concept_Analysis');  TS = fullfile(root, 'Turbofan_Sizing');
set(0, 'DefaultFigureVisible', 'off');
addpath(CA);   % concept_constraint_check is used while working in Turbofan_Sizing
sets = struct('tag', {'rfp', 'team'}, 'ts', {'rfp_requirements', 'requirements'});
for n = [2 3 5 6]
    od = fullfile(CA, 'output', sprintf('concept%d', n));
    if ~exist(od, 'dir'), mkdir(od); end
    S = table();
    for s = sets
        % ---------------- fixed wing ----------------
        cd(CA);
        f1 = sprintf('concept%d_%s.json', n, s.tag);
        txt = evalc('rc = concept_sizing(f1, ''history'', true, ''plot'', false);');
        base = fullfile(od, sprintf('concept%d_%s_fixed_wing', n, s.tag));
        write_txt_(base + "_output.txt", sprintf('FIXED wing (VSP S_ref held), %s\nInput: Concept_Analysis/inputs/%s\n\n', s.tag, f1), txt);
        writetable(req_table_(rc.check), base + "_requirements.csv");
        writetable(weights_table_(rc.OEW_breakdown, rc.W_payload, rc.W_fuel, rc.W_TO), base + "_weights.csv");
        if rc.converged, st = "converged"; else, st = "NOT converged (last iterate)"; end
        S = [S; summary_row_(s.tag, "fixed", st, f1, rc.W_TO, rc.S_ref, rc.T_SL, rc.W_fuel, rc.OEW, ...
                             rc.W_payload, rc.governing_mission, rc.CD0, rc.e, rc.check)]; %#ok<AGROW>

        % ---------------- rubber wing ----------------
        cd(TS);
        f2 = sprintf('prm2_concept%d_%s.json', n, s.ts);
        base = fullfile(od, sprintf('concept%d_%s_rubber_wing', n, s.tag));
        try
            txt = evalc('r = run_tf_sizing(''json'', f2, ''plot'', false, ''history'', true);');
            C = concept_constraint_check(r.obj, r.W_TO);
            write_txt_(base + "_output.txt", sprintf('RUBBER wing (loop sizes S_ref), %s\nInput: Turbofan_Sizing/%s\n\n', s.tag, f2), txt);
            writetable(req_table_(C), base + "_requirements.csv");
            writetable(weights_table_(r.OEW_breakdown, r.W_payload, r.W_fuel, r.W_TO), base + "_weights.csv");
            if r.converged, st = "converged"; else, st = "NOT converged (loop hit max_iter; last iterate)"; end
            S = [S; summary_row_(s.tag, "rubber", st, f2, r.W_TO, r.S_ref, r.T_SL, r.W_fuel, r.OEW, ...
                                 r.W_payload, r.governing_mission, r.obj.aero.CD0, r.obj.aero.e, C)]; %#ok<AGROW>
        catch e
            % Not converged: report the failure, then the requirements at the
            % fixed-wing W_TO with the wing at the least-thrust W/S (the same
            % reference airplane as the _NOT_CONVERGED diagram).
            W = rc.W_TO;
            o = tf_disciplines(f2);  o.prop.T_SL = o.prop.T_SL_fixed();  o.geom.S_ref = rc.S_ref;
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
            C = concept_constraint_check(o, W);
            bd = o.wts.OEW_breakdown(W);
            hdr = sprintf(['RUBBER wing (loop sizes S_ref), %s\nInput: Turbofan_Sizing/%s\n\n', ...
                '*** THE SIZING LOOP DID NOT CONVERGE ***\n%s\n\n', ...
                'REFERENCE POINT (not a closed design): the requirements evaluated at the fixed-wing W_TO\n', ...
                '%.0f lbf with the wing at the least-thrust W/S (S_ref %.1f ft^2) and the F124 (T_SL %.0f lbf).\n', ...
                'The requirements table and weights below belong to that reference airplane; no mission\n', ...
                'fuel was closed, so W_fuel is W_TO - OEW - payload at that weight.\n\n'], ...
                s.tag, f2, e.message, W, o.geom.S_ref, o.prop.T_SL);
            write_txt_(base + "_NOT_CONVERGED_output.txt", hdr, evalc('disp(req_table_(C))'));
            writetable(req_table_(C), base + "_NOT_CONVERGED_requirements.csv");
            Wf = W - bd.total - o.wts.payload();
            writetable(weights_table_(bd, o.wts.payload(), Wf, W), base + "_NOT_CONVERGED_weights.csv");
            S = [S; summary_row_(s.tag, "rubber", "NOT converged (reference point)", f2, W, o.geom.S_ref, o.prop.T_SL, ...
                                 Wf, bd.total, o.wts.payload(), "-", o.aero.CD0, o.aero.e, C)]; %#ok<AGROW>
        end
        close all
    end
    writetable(S, fullfile(od, sprintf('concept%d_summary.csv', n)));
    fprintf('concept %d: exported %d cases to %s\n', n, height(S), od);
end
set(0, 'DefaultFigureVisible', 'on');

%% ------------------------------------------------------------------ helpers
function write_txt_(path, header, body)
    fid = fopen(path, 'w');
    fprintf(fid, '%s', header);
    body = regexprep(body, '<a href="[^"]*"[^>]*>|</a>', '');   % strip MATLAB hyperlinks
    fprintf(fid, '%s', body);
    fclose(fid);
end

function T = req_table_(C)
    T = C.table;
    T.Properties.VariableNames = {'requirement', 'type', 'required', 'available', 'margin_pct', 'pass'};
end

function T = weights_table_(bd, W_pay, W_fuel, W_TO)
    item = ["wing"; "horizontal_tail"; "vertical_tail"; "fuselage"; "landing_gear"; ...
            "installed_engine"; "all_else_empty"; "OEW"; "payload"; "fuel"; "W_TO"];
    lbf  = [bd.wing; bd.horizontal_tail; bd.vertical_tail; bd.fuselage; bd.landing_gear; ...
            bd.installed_engine; bd.all_else_empty; bd.total; W_pay; W_fuel; W_TO];
    if isfield(bd, 'v_tail')
        item = [item; "v_tail_total (= horizontal + vertical rows)"];  lbf = [lbf; bd.v_tail];
    end
    T = table(item, lbf, 100 * lbf / W_TO, 'VariableNames', {'item', 'lbf', 'pct_of_W_TO'});
end

function R = summary_row_(setname, wing, status, file, W, S, T, Wf, OEW, Wp, gov, CD0, e, C)
    tbl = C.table;
    failed = strjoin(string(tbl.requirement(~tbl.pass)), '; ');
    if failed == "", failed = "none"; end
    R = table(string(setname), string(wing), string(status), string(file), W, S, W / S, T, T / W, ...
              Wf, OEW, Wp, string(gov), CD0, e, C.all_pass, failed, string(C.worst), min(tbl.margin_pct), ...
              C.W_max, C.growth_pct, ...
        'VariableNames', {'requirements', 'wing', 'status', 'input_file', 'W_TO_lbf', 'S_ref_ft2', 'WS_psf', ...
                          'T_SL_lbf', 'TW', 'W_fuel_lbf', 'OEW_lbf', 'W_payload_lbf', 'governing_mission', ...
                          'CD0', 'e', 'all_pass', 'failed', 'worst', 'worst_margin_pct', 'W_max_lbf', 'growth_pct'});
end
