function [json_out, tag] = tf_json_with_overrides(json_path, airfoil, sweep_deg)
%TF_JSON_WITH_OVERRIDES  Copy of a requirements file with the wing airfoil and/or sweep changed.
%
%   [json_out, tag] = tf_json_with_overrides(json_path, airfoil, sweep_deg)
%
%   Inputs
%     json_path  requirements file to start from
%     airfoil    "supercritical", "conventional", "custom", or "" to keep
%                the file's own aerodynamics.airfoil_type
%     sweep_deg  quarter-chord sweep [deg], or [] to keep the file's own
%                geometry.sweep_quarter_chord_deg
%   Outputs
%     json_out   path of the modified copy (in tempdir), or json_path itself
%                when nothing is overridden
%     tag        short text for file names, e.g. "_conventional_35deg"
%                ("" when nothing is overridden)
%
%   WHY A COPY. Every discipline class (TfProp, TfGeom, TfAero, ...) and the
%   constraint set read the JSON from its path. Writing a modified copy and
%   passing ITS path on is the one change that reaches all of them at once,
%   with no change to the classes. The original file is never touched.
%
%   What the two overrides do downstream (see TfAero):
%     airfoil   sets the Korn technology factor: supercritical 0.95,
%               conventional 0.87 -> drag-divergence Mach -> wave drag in the
%               max-speed, cruise, Ps and turn conditions and in the mission.
%     sweep     moves the drag-divergence Mach (Korn, through cos L), the
%               leading-edge sweep and so the Oswald-efficiency formula, the
%               tail areas (span and MAC change with nothing else), and -
%               when the file has aerodynamics.CLmax_reference_sweep_deg -
%               scales all three CLmax values by cos(L)/cos(L_ref).
%   NOT modelled: the wing-weight penalty of sweep (the unit-weight method
%   ignores it) and the airfoil's handling effects.
%
%   Used by run_tf_constraints, run_tf_sizing ('airfoil', 'sweep_deg'
%   options) and airfoil_sweep_study.

    if nargin < 2 || isempty(airfoil), airfoil = ""; end
    if nargin < 3, sweep_deg = []; end
    airfoil = lower(string(airfoil));

    tag = "";
    if airfoil == "" && isempty(sweep_deg)
        json_out = string(json_path);
        return
    end

    J = jsondecode(fileread(json_path));

    if airfoil ~= ""
        if ~ismember(airfoil, ["supercritical", "conventional", "custom"])
            error('tf_json_with_overrides:UndefinedAirfoil', ...
                'airfoil must be "supercritical", "conventional" or "custom", not "%s".', airfoil);
        end
        J.aerodynamics.airfoil_type = char(airfoil);
        % A stale korn_technology_factor would only trigger the mismatch
        % warning in TfAero; drop it unless the user asked for "custom".
        if airfoil ~= "custom" && isfield(J.aerodynamics, 'korn_technology_factor')
            J.aerodynamics = rmfield(J.aerodynamics, 'korn_technology_factor');
        end
        tag = tag + "_" + airfoil;
    end

    if ~isempty(sweep_deg)
        if ~(isscalar(sweep_deg) && isnumeric(sweep_deg) && sweep_deg >= 0 && sweep_deg < 60)
            error('tf_json_with_overrides:BadSweep', 'sweep_deg must be a number in [0, 60).');
        end
        J.geometry.sweep_quarter_chord_deg = sweep_deg;
        tag = tag + "_" + strrep(sprintf('%gdeg', sweep_deg), '.', 'p');
    end

    [~, base] = fileparts(json_path);
    json_out = string(fullfile(tempdir, base + tag + ".json"));
    fid = fopen(json_out, 'w');
    if fid < 0
        error('tf_json_with_overrides:CannotWrite', 'Cannot write %s.', json_out);
    end
    closer = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', jsonencode(J, 'PrettyPrint', true));
end
