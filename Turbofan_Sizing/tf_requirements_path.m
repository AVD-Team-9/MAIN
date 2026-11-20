function [p] = tf_requirements_path()
%TF_REQUIREMENTS_PATH  Absolute path to the turbofan requirements JSON.
%
%   p = tf_requirements_path() returns the full path of tf_requirements.json,
%   the one file every turbofan class and driver reads.
%
%   This replaces ttpa_requirements_path from the TTPA code. It looks in the
%   folder that holds THIS .m file, not in MATLAB's current folder, so the
%   code works no matter where you call it from - as long as this folder is
%   the one on the path.
%
%   DO NOT put this folder and the parent IHW3_Code folder on the MATLAB path
%   at the same time. Several file names (get_state, get_con, run_constraints,
%   matching_envelope, sizing_pass, ...) exist in both, and MATLAB would pick
%   whichever folder is higher on the path. cd into this folder and run from
%   here.

    % Folder that holds this file
    here = fileparts(mfilename('fullpath'));

    % The requirements file sits next to it
    p = string(fullfile(here, 'tf_requirements.json'));

    if ~isfile(p)
        error('tf_requirements_path:notFound', ...
            'tf_requirements.json was not found next to %s.', mfilename('fullpath'));
    end
end
