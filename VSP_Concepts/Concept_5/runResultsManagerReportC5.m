% runResultsManagerReportV3.m
clear; clc; close all

%% Manual inputs
taperRatio   = 0.66550;    % Overall wing tip chord / root chord
clmaxSection = 1.65;   % Estimated 2D airfoil maximum lift coefficient
sweepQC_deg = 17.20000;      % Current concept's quarter-chord sweep

%% Project settings
files.low      = 'lowspeedc5.csv';
files.cruise   = 'cruisec5.csv';
files.parasite = 'parasitec5.csv';
files.geometry = 'geomc5.csv';


out = fullfile(pwd, 'aero_reportc5');

if ~isfolder(out), mkdir(out); end

%% Read the two VSPAERO exports and the cruise reference dimensions
low = readVSPResultsCSV(files.low);
cruise = readVSPResultsCSV(files.cruise);

sLow    = historyValue(files.low, "FC_Sref_");
sCruise = historyValue(files.cruise, "FC_Sref_");
bCruise = historyValue(files.cruise, "FC_Bref_");
AR = NaN;
if sCruise > 0 && bCruise > 0
    AR = bCruise^2 / sCruise;
end

%% Cruise parasite drag: treat the export as Mach 0.4
[cd0Export, sParasite] = readParasite(files.parasite);
swet = readWetArea(files.geometry);

cd0Cruise = NaN;
if cd0Export > 0 && sParasite > 0 && sCruise > 0
    cd0Cruise = cd0Export * sParasite / sCruise;
end

lowCD = column(low, 'CDwtot');
cruiseCD = column(cruise, 'CDwtot');
cruiseSource = 'exported CDwtot';
cruiseCDiw = column(cruise, 'CDiw');

if isfinite(cd0Cruise) && any(isfinite(cruiseCDiw))
    cruiseCD = cruiseCDiw + cd0Cruise;
    cruiseSource = 'CDiw + rescaled parasite CD0';
end

%% Sizing JSON values
J = struct;
J.Cfe = NaN;
J.CD0_reference = cd0Cruise;
J.airfoil_type = 'conventional';

if isfinite(cd0Cruise) && swet > 0
    J.Cfe = cd0Cruise * sCruise / swet;
end

% Existing preliminary high-lift assumptions.
J.CLmax_clean = 0.9 * clmaxSection * cosd(sweepQC_deg);
deltaCLlanding = 1.30 * 0.45 * cosd(20);
J.CLmax_takeoff = J.CLmax_clean + 0.70 * deltaCLlanding;
J.CLmax_landing = J.CLmax_clean + deltaCLlanding;
J.CLmax_reference_sweep_deg = sweepQC_deg;
J.delta_CD0_flaps_takeoff = 0.02;
J.delta_CD0_flaps_landing = 0.07;
J.delta_CD0_gear_down = 0.02;
J.delta_CD0_windmilling_engine = 0;  % Current single-engine concept

sweepLE_deg = NaN;
eClean = NaN;
if isfinite(taperRatio) && taperRatio > 0 && isfinite(AR)
    sweepLE_deg = atand(tand(sweepQC_deg) + ...
        (1 - taperRatio) / (AR * (1 + taperRatio)));
    q = 1 - 0.045 * AR^0.68;
    if sweepLE_deg < 30
        eClean = 1.78 * q - 0.64;
    else
        eClean = 4.61 * q * cosd(sweepLE_deg)^0.15 - 3.1;
    end
    if eClean <= 0 || eClean > 1
        eClean = NaN;
    end
end
J.delta_e_flaps_takeoff = -0.05 * eClean;
J.delta_e_flaps_landing = -0.10 * eClean;

% Do not put unresolved values in the JSON that gets copied into sizing.
names = fieldnames(J);
missing = cellfun(@(n) isnumeric(J.(n)) && ~isfinite(J.(n)), names);
ready = rmfield(J, names(missing));

%% Fit both polars using the rows in the CSVs
cases = [
    analyze(low, lowCD, 'Low speed', 'exported CDwtot')
    analyze(cruise, cruiseCD, 'Cruise', cruiseSource)
];

%% Write the report before making plots
reportPath = fullfile(out, 'Aerodynamics_Report.txt');
[fid, msg] = fopen(reportPath, 'w');
if fid < 0, error('Cannot write report: %s', msg); end

fprintf(fid, 'AERODYNAMIC DESIGN REPORT\n');
fprintf(fid, 'AoA comes from each VSPAERO export. No manual fit window.\n');
fprintf(fid, 'Fits use all usable exported rows and are drawn over sampled CL.\n');
fprintf(fid, 'Parasite export treated as Mach 0.4; no Mach matching.\n');
fprintf(fid, 'Low-speed Sref: %.9g\nCruise Sref: %.9g\n', sLow, sCruise);
fprintf(fid, 'Parasite Sref: %.9g\nGeometry wetted area: %.9g\n', sParasite, swet);
fprintf(fid, 'Exported parasite CD0: %.9g\nCruise-basis CD0: %.9g\n', ...
    cd0Export, cd0Cruise);
fprintf(fid, 'Cruise AR: %.9g\nTaper ratio: %.9g\n', AR, taperRatio);
fprintf(fid, 'Section clmax estimate: %.9g\nQuarter-chord sweep: %.9g deg\n', ...
    clmaxSection, sweepQC_deg);
fprintf(fid, 'Leading-edge sweep: %.9g deg\nClean Oswald efficiency: %.9g\n', ...
    sweepLE_deg, eClean);

for k = 1:numel(cases)
    F = cases(k);
    fprintf(fid, '\n%s | Mach %.5g | Re %.6g\n', F.label, F.mach, F.re);
    fprintf(fid, 'Drag source: %s\n', F.source);
    fprintf(fid, 'Exported rows: %d | Usable drag rows: %d\n', ...
        numel(F.alpha), F.nDrag);
    fprintf(fid, 'Lift slope: %.9g per deg | Zero-lift AoA: %.9g deg\n', ...
        F.liftSlope, F.zeroLift);
    fprintf(fid, 'Cm slope: %.9g per deg\n', F.cmSlope);
    fprintf(fid, 'Best sampled L/D: %.9g at CL %.9g, AoA %.9g deg\n', ...
        F.bestLD, F.bestCL, F.bestAlpha);
    if isempty(F.fitCL)
        fprintf(fid, 'No quadratic: need three distinct usable CL values.\n');
    else
        fprintf(fid, 'CD = b0 + b1*CL + b2*CL^2\n');
        fprintf(fid, 'b0: %.9g | b1: %.9g | b2: %.9g\n', ...
            F.coeff(3), F.coeff(2), F.coeff(1));
    end
end

if any(missing)
    fprintf(fid, '\nUNRESOLVED JSON INPUTS (leave existing values unchanged)\n');
    fprintf(fid, '%s\n', names{missing});
end
fprintf(fid, '\nJSON INPUTS ONLY | merge into aerodynamics\n');
fprintf(fid, '%s\n', jsonencode(ready, 'PrettyPrint', true));
fclose(fid);
fprintf('Report written: %s\n', reportPath);

%% Plot the comparison and export a PNG
colors = [0.11 0.36 0.65; 0.76 0.31 0.12];
fig = figure('Name', 'Aerodynamic comparison', 'Color', 'w', ...
    'Position', [70 70 1150 760]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = panel(layout, 'Angle of attack (deg)', 'C_L', 'Lift curve');
ax2 = panel(layout, 'C_D', 'C_L', 'Clean drag polar');
ax3 = panel(layout, 'C_L', 'L/D', 'Sampled efficiency');
ax4 = panel(layout, 'Angle of attack (deg)', 'C_m', 'Pitching moment');

for k = 1:numel(cases)
    F = cases(k);
    c = colors(k,:);
    label = sprintf('%s | M %.3g', F.label, F.mach);

    plot(ax1, F.alpha, F.cl, '-o', 'Color', c, 'DisplayName', label);
    plot(ax2, F.cd, F.cl, 'o', 'Color', c, 'LineStyle', 'none', ...
        'DisplayName', [F.label ' samples']);
    if ~isempty(F.fitCL)
        plot(ax2, F.fitCD, F.fitCL, '-', 'Color', c, 'LineWidth', 2, ...
            'DisplayName', [F.label ' quadratic']);
    end
    plot(ax3, F.cl, F.ld, '-o', 'Color', c);
    plot(ax4, F.alpha, F.cm, '-o', 'Color', c);
end
legend(ax1, 'Location', 'best', 'Box', 'off');
legend(ax2, 'Location', 'best', 'Box', 'off');
exportgraphics(fig, fullfile(out, 'Aerodynamics_Comparison.png'), ...
    'Resolution', 300);

%% Local functions
function F = analyze(T, cd, label, source)
    [a, order] = sort(column(T, 'Alpha'));
    cl = column(T, 'CLwtot'); cl = cl(order);
    cm = column(T, 'CMytot'); cm = cm(order);
    cd = cd(order);

    F.label = label;
    F.source = source;
    F.mach = firstNumber(column(T, 'Mach'));
    F.re = firstNumber(column(T, 'ReCref'));
    F.alpha = a;
    F.cl = cl;
    F.cd = cd;
    F.cm = cm;

    good = isfinite(a) & isfinite(cl) & isfinite(cd) & cd > 0;
    F.nDrag = nnz(good);
    F.ld = nan(size(cd));
    F.ld(good) = cl(good) ./ cd(good);
    F.bestLD = NaN;
    F.bestCL = NaN;
    F.bestAlpha = NaN;

    ids = find(good & cl > 0);
    if ~isempty(ids)
        [F.bestLD, j] = max(F.ld(ids));
        F.bestCL = cl(ids(j));
        F.bestAlpha = a(ids(j));
    end

    p = lineFit(a, cl);
    F.liftSlope = p(1);
    F.zeroLift = NaN;
    if abs(p(1)) > eps, F.zeroLift = -p(2) / p(1); end
    p = lineFit(a, cm);
    F.cmSlope = p(1);

    F.fitCL = [];
    F.fitCD = [];
    F.coeff = [NaN NaN NaN];
    x = cl(good);
    if numel(unique(x)) >= 3
        F.coeff = polyfit(x, cd(good), 2);
        F.fitCL = linspace(min(x), max(x), 300);
        F.fitCD = polyval(F.coeff, F.fitCL);
    end
end

function p = lineFit(x, y)
    use = isfinite(x) & isfinite(y);
    p = [NaN NaN];
    if numel(unique(x(use))) >= 2
        p = polyfit(x(use), y(use), 1);
    end
end

function x = column(T, name)
    x = nan(height(T), 1);
    if ismember(name, T.Properties.VariableNames)
        x = T.(name);
        x = x(:);
    end
end

function v = firstNumber(x)
    v = NaN;
    i = find(isfinite(x), 1);
    if ~isempty(i), v = x(i); end
end

function v = historyValue(file, field)
    parts = splitlines(string(fileread(file)));
    v = NaN;
    for i = 1:numel(parts)
        row = split(parts(i), ',');
        if numel(row) >= 2 && key(row(1)) == key(field)
            v = number(row(2));
            if isfinite(v), return; end
        end
    end
end

function [cd0, sref] = readParasite(file)
    cd0 = NaN;
    sref = NaN;
    if ~isfile(file), return; end

    A = readcell(file, 'Delimiter', ',');
    if isempty(A), return; end
    K = string(cellfun(@key, A, 'UniformOutput', false));

    [r, c] = find(K == "srefft2", 1);
    if ~isempty(r) && r < size(A, 1)
        sref = number(A{r+1, c});
    end

    h = find(K(:,1) == "componentname", 1);
    if isempty(h), return; end
    c = find(K(h,:) == "cd", 1);
    [r, ~] = find(K == "total" | K == "totals");
    r = unique(r(r > h));
    if ~isempty(c) && isscalar(r)
        cd0 = number(A{r, c});
    end
end

function swet = readWetArea(file)
    swet = NaN;
    if ~isfile(file), return; end

    A = readcell(file, 'Delimiter', ',');
    if isempty(A), return; end
    K = string(cellfun(@key, A, 'UniformOutput', false));
    h = find(K(:,1) == "name", 1);
    if isempty(h), return; end
    c = find(K(h,:) == "wetarea", 1);
    rows = (1:size(A,1))';
    r = find(rows > h & (K(:,1) == "total" | K(:,1) == "totals"), 1);
    if ~isempty(r) && ~isempty(c)
        swet = number(A{r, c});
    end
end

function k = key(x)
    k = "";
    if (ischar(x) || isstring(x)) && ~ismissing(string(x))
        k = regexprep(lower(string(x)), '[^a-z0-9]', '');
    end
end

function v = number(x)
    if isnumeric(x)
        v = double(x);
    else
        v = str2double(strip(erase(string(x), '"')));
    end
end

function ax = panel(layout, xl, yl, ttl)
    ax = nexttile(layout);
    hold(ax, 'on');
    grid(ax, 'on');
    xlabel(ax, xl);
    ylabel(ax, yl);
    title(ax, ttl);
end