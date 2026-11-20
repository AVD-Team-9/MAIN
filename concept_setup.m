function json_path = concept_setup(json_path)
%CONCEPT_SETUP  Put the Turbofan_Sizing library on the path and resolve an input file.
%
%   concept_setup                 path only
%   f = concept_setup(f)          also resolves f: as given, else in inputs/
%
%   WHY. The concept analysis REUSES the Turbofan_Sizing classes and
%   functions (TfProp, TfGeom, TfAero, TfWeights, the constraints, the
%   mission segments) instead of copying them, so a fix made there applies
%   here too. Every file in this folder starts with "concept_", so nothing
%   here shadows a Turbofan_Sizing file.
%
%   The parent IHW3_Code folder (the TTPA code) must NOT be on the path at
%   the same time: it has its own get_state, get_con, run_constraints, ...
%   with the same names. This function stops with instructions if it is.

    here = fileparts(mfilename('fullpath'));
    lib  = fullfile(fileparts(here), 'Turbofan_Sizing');
    if ~isfolder(lib)
        error('concept_setup:NoLibrary', 'Turbofan_Sizing folder not found next to Concept_Analysis (%s).', lib);
    end
    if ~contains([pathsep, path, pathsep], [pathsep, lib, pathsep])
        addpath(lib);
    end
    w = which('get_state');
    if ~startsWith(w, lib)
        error('concept_setup:Shadowed', ['get_state resolves to "%s", not to Turbofan_Sizing. The parent ', ...
            'IHW3_Code (TTPA) folder is probably on the path: run rmpath(''%s'') or restoredefaultpath, ', ...
            'then try again.'], w, fileparts(here));
    end

    if nargin > 0
        json_path = string(json_path);
        if ~isfile(json_path) && isfile(fullfile(here, 'inputs', json_path))
            json_path = string(fullfile(here, 'inputs', json_path));
        end
        if ~isfile(json_path)
            error('concept_setup:NoFile', 'Input file "%s" not found (looked here and in inputs/).', json_path);
        end
    end
end
