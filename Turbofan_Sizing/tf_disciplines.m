function [obj] = tf_disciplines(json_path)
%TF_DISCIPLINES  Build the turbofan discipline bundle.
%
%   obj = tf_disciplines()            reads tf_requirements.json
%   obj = tf_disciplines(json_path)   reads another requirements file
%
%   Returns the same struct layout ttpa_disciplines returned, so every
%   function that takes "obj" works the same way:
%
%       obj.prop   TfProp        thrust lapse, TSFC, engine weight and size.
%                                Receives T_SL (replaces P_SL).
%       obj.geom   TfGeom        the Level-2 airplane. Receives S_ref.
%       obj.aero   TfAero        CD0 from the geometry, swept-wing e, wave drag
%       obj.wts    TfWeights     Raymer Table 15.2 transport build-up
%       obj.miss   struct        mission profile (MissionProfileReader)
%       obj.cons   struct array  constraint conditions (ConstraintSetImporter)
%
%   BUILD ORDER MATTERS, for the same reason as in ttpa_disciplines: each
%   constructor takes the objects it depends on.
%
%       prop                   depends on nothing
%       geom (json, prop)      nacelle length AND diameter follow the engine
%       aero (json, geom)      CD0 = Cfe S_wet/S_ref, sweep, t/c
%       wts  (json, geom, prop)  areas and engine weight
%
%   All four are HANDLE objects: writing obj.prop.T_SL or obj.geom.S_ref
%   resizes the airplane for every discipline at once.

    if nargin < 1
        json_path = tf_requirements_path();
    end

    prop = TfProp(json_path);
    geom = TfGeom(json_path, prop);
    aero = TfAero(json_path, geom);
    wts  = TfWeights(json_path, geom, prop);

    % Sizing missions: sizing.missions (a list of names) if the file gives
    % one, otherwise every profile in the missions block. mission_fuel flies
    % them all and sizes to the one that needs the most fuel. obj.miss stays
    % the FIRST one, for code that only looks at a single mission.
    J = jsondecode(fileread(json_path));
    if isfield(J, 'sizing') && isfield(J.sizing, 'missions') && ~isempty(J.sizing.missions)
        names = string(J.sizing.missions);
    else
        f = string(fieldnames(J.missions));
        names = f(~startsWith(f, "_") & ~startsWith(f, "x_"));
        names = [names(names == "std_mission"); names(names ~= "std_mission")];   % std first
    end
    missions = cell(1, numel(names));
    for i = 1:numel(names)
        missions{i} = MissionProfileReader.read_profile(json_path, names(i));
    end
    miss = missions{1};
    cons = ConstraintSetImporter.read_conditions(json_path);

    obj = struct('aero', aero, 'prop', prop, 'wts', wts, 'geom', geom, ...
                 'miss', miss, 'cons', cons);
    obj.missions = missions;            % (assigned after: struct() would split a cell)
end
