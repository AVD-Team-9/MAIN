function T = airfoil_sweep_study(varargin)
%AIRFOIL_SWEEP_STUDY  Trade airfoil type and wing sweep: size the airplane for every pair.
%
%   T = airfoil_sweep_study('json', 'prm2_m090_requirements.json')
%   T = airfoil_sweep_study('json', f, 'sweeps', 20:5:40, ...
%                           'airfoils', ["conventional" "supercritical"], ...
%                           'mode', "sizing", 'plot', true, 'save_png', true)
%
%   For every airfoil type and every quarter-chord sweep it runs either the
%   full sizing loop (mode "sizing", default) or the fixed-weight constraint
%   analysis (mode "constraints", much faster), on a modified copy of the
%   requirements file (tf_json_with_overrides - the original is not changed),
%   and tabulates the result.
%
%   OPTIONS (name/value)
%     'json'      requirements file (default tf_requirements.json)
%     'sweeps'    quarter-chord sweeps [deg] (default 15:5:40)
%     'airfoils'  string array (default ["conventional" "supercritical"])
%     'mode'      "sizing" (W_TO, S_ref, T_SL all closed) or "constraints"
%                 (corner T/W and W/S at the file's W_TO_estimate_lbf)
%     'plot'      true (default) draws the trade plots
%     'save_png'  false (default); true writes output/<prefix>_airfoil_sweep_study.png
%
%   OUTPUT  T  table, one row per (airfoil, sweep):
%     airfoil, sweep_qc_deg, sweep_LE_deg, oswald_e, CLmax_clean,
%     Mdd_maxspeed    Korn M_dd at the CL of the FASTEST flight condition
%                     (the max-speed requirement) - compare with its Mach
%     max_speed_mach  that condition's Mach
%     TW_maxspeed     T/W the max-speed condition needs at the corner W/S
%     WS, TW          corner (constraints) or converged (sizing) W/S and T/W
%     W_TO, S_ref, T_uninstalled_per_engine, W_fuel   (sizing mode;
%                     constraints mode gives W_TO = the file's estimate,
%                     S_ref = W_TO/WS and T = TW W_TO / (1 - loss), fuel NaN)
%     driver          the requirement that sizes the engine
%     status          "ok" or the error message
%
%   WHAT THE TWO VARIABLES CHANGE (see TfAero and tf_json_with_overrides)
%     airfoil  Korn kappa (supercritical 0.95, conventional 0.87) -> M_dd
%     sweep    M_dd (through cos L), Oswald e (through the LE sweep), tail
%              areas, and CLmax (by cos L/cos L_ref) IF the file gives
%              aerodynamics.CLmax_reference_sweep_deg. Without that key a
%              swept wing keeps its unswept CLmax and the study favours sweep
%              unfairly - the function warns.
%   NOT MODELLED: the wing-weight penalty of sweep and the handling
%   qualities of either airfoil. Treat the trend, not the optimum, as the
%   result, and check the chosen wing in your S&C analysis.

    p = inputParser;
    p.addParameter('json', tf_requirements_path());
    p.addParameter('sweeps', 15:5:40, @(x) isnumeric(x) && ~isempty(x));
    p.addParameter('airfoils', ["conventional" "supercritical"]);
    p.addParameter('mode', "sizing", @(x) ismember(string(x), ["sizing" "constraints"]));
    p.addParameter('plot', true);
    p.addParameter('save_png', false);
    p.parse(varargin{:});
    A = p.Results;
    json_path = string(A.json);
    airfoils  = string(A.airfoils);
    mode      = string(A.mode);

    J0 = jsondecode(fileread(json_path));
    if ~isfield(J0.aerodynamics, 'CLmax_reference_sweep_deg')
        warning('airfoil_sweep_study:NoCLmaxSweepReference', ...
            ['%s has no aerodynamics.CLmax_reference_sweep_deg, so CLmax will NOT fall with ', ...
             'sweep and swept wings will look better than they are.'], json_path);
    end

    % The fastest flight condition is taken as the max-speed requirement.
    cons = ConstraintSetImporter.read_conditions(json_path);
    i_ms = []; M_ms = -Inf;
    for k = 1:numel(cons)
        c = get_con(k, cons);
        if c.type == "flight" && c.mach > M_ms, M_ms = c.mach; i_ms = k; end
    end

    rows = {};
    ws0 = warning('off', 'TfProp:ThrustOutOfRange');
    restore = onCleanup(@() warning(ws0));
    for a = airfoils
        for L = A.sweeps
            fprintf('  %-14s %5.1f deg ... ', a, L);
            row = struct('airfoil', a, 'sweep_qc_deg', L, 'sweep_LE_deg', NaN, 'oswald_e', NaN, ...
                'CLmax_clean', NaN, 'Mdd_maxspeed', NaN, 'max_speed_mach', M_ms, 'TW_maxspeed', NaN, ...
                'WS', NaN, 'TW', NaN, 'W_TO', NaN, 'S_ref', NaN, 'T_uninstalled_per_engine', NaN, ...
                'W_fuel', NaN, 'driver', "", 'status', "ok");
            try
                f = tf_json_with_overrides(json_path, a, L);
                if mode == "sizing"
                    [~, r] = evalc('run_tf_sizing(''json'', f, ''plot'', false, ''history'', false)');
                    obj = r.obj;
                    row.WS = r.W_TO / r.S_ref;   row.TW = r.T_SL / r.W_TO;
                    row.W_TO = r.W_TO;           row.S_ref = r.S_ref;
                    row.T_uninstalled_per_engine = r.engine.T_uninstalled_per_engine;
                    row.W_fuel = r.W_fuel;       row.driver = string(r.driving);
                else
                    [~, R] = evalc('run_tf_constraints(''json'', f, ''plot'', false)');
                    obj = R.obj;
                    row.WS = R.WS_best;          row.TW = R.TW_best;
                    row.W_TO = R.W_TO;           row.S_ref = R.S_ref;
                    row.T_uninstalled_per_engine = R.TW_best * R.W_TO / obj.prop.n_engines ...
                                                   / (1 - obj.prop.installation_loss);
                    [TWl, ~, nm] = run_constraints(row.WS, obj);
                    [~, ix] = max(TWl);  row.driver = nm(ix);
                end
                row.sweep_LE_deg = obj.geom.sweep_LE_deg;
                row.oswald_e     = obj.aero.e;
                row.CLmax_clean  = obj.aero.CLmax_clean;
                if ~isempty(i_ms)
                    c  = get_con(i_ms, obj.cons);
                    st = get_state(c.alt, c.mach);
                    row.Mdd_maxspeed = obj.aero.drag_divergence_mach(c.n * c.beta * row.WS / st.q);
                    row.TW_maxspeed  = constraint_flight(row.WS, obj, i_ms);
                end
                fprintf('W %6.0f  T %5.0f  M_dd %.3f  %s\n', row.W_TO, row.T_uninstalled_per_engine, ...
                        row.Mdd_maxspeed, row.driver);
            catch ME
                row.status = string(ME.message);
                fprintf('FAILED: %s\n', ME.message);
            end
            rows{end+1} = row; %#ok<AGROW>
        end
    end
    T = struct2table([rows{:}]);

    if A.plot
        fig = figure('Name', 'Airfoil and sweep trade', 'Color', 'w', 'Position', [100 100 1150 700]);
        tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact');
        title(tl, sprintf('Airfoil / sweep trade (%s mode) - %s', mode, ...
              erase(string(fileparts_name(json_path)), "_requirements")), 'Interpreter', 'none');
        ok = T.status == "ok";
        panels = {'W_TO', 'Takeoff weight W_{TO} [lbf]'; ...
                  'T_uninstalled_per_engine', 'Uninstalled thrust per engine [lbf]'; ...
                  'Mdd_maxspeed', 'M_{dd} at the max-speed CL'; ...
                  'CLmax_clean', 'Clean CL_{max} (sweep-corrected)'};
        for k = 1:4
            ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
            for a = airfoils
                m = ok & T.airfoil == a;
                plot(ax, T.sweep_qc_deg(m), T.(panels{k,1})(m), '-o', 'LineWidth', 1.5, 'DisplayName', char(a));
            end
            if strcmp(panels{k,1}, 'Mdd_maxspeed') && isfinite(M_ms)
                yline(ax, M_ms, '--k', sprintf('max-speed requirement M %.2f', M_ms), 'HandleVisibility', 'off');
            end
            xlabel(ax, 'quarter-chord sweep [deg]'); ylabel(ax, panels{k,2});
            if k == 1, legend(ax, 'Location', 'best'); end
        end
        if A.save_png
            out = fullfile(fileparts(mfilename('fullpath')), 'output');
            if ~exist(out, 'dir'), mkdir(out); end
            exportgraphics(fig, fullfile(out, erase(fileparts_name(json_path), "_requirements") ...
                + "_airfoil_sweep_study.png"), 'Resolution', 200);
        end
    end
end

function n = fileparts_name(p)
    [~, n] = fileparts(p);
    n = string(n);
end
