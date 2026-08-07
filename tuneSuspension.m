function [bestKs, bestCs, tuningTable] = tuneSuspension()
% TUNESUSPENSION Parameter sweep over suspension stiffness (Ks) and
% damping (Cs), scoring every combination across all 5 road cases in
% roadSuite(), and selecting the design that meets constraints (see
% scoreSuspension.m) while minimizing mean comfort (RMS body
% acceleration) across cases.
%
% Uses Simulink.SimulationInput + parsim (Parallel Computing Toolbox)
% if available, falling back to a serial sim() loop otherwise.
%
% Saves results/tuning_heatmap.png, results/tuning_comparison.png, and
% results/tuning_table.csv.

addpath(fullfile(fileparts(mfilename('fullpath')), 'tools'));

mdl = 'model';
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

KsGrid = 15000:5000:60000;   % N/m
CsGrid = 500:250:2500;       % Ns/m
nKs = numel(KsGrid);
nCs = numel(CsGrid);

cases = roadSuite();
nCases = numel(cases);

fprintf('Sweeping %d Ks values x %d Cs values x %d road cases = %d simulations...\n', ...
    nKs, nCs, nCases, nKs*nCs*nCases);

% --- Build flattened SimulationInput array ---------------------------
nTotal = nKs * nCs * nCases;
inputs = Simulink.SimulationInput.empty(0, 1);
flatKi = zeros(nTotal, 1);
flatCi = zeros(nTotal, 1);
flatJi = zeros(nTotal, 1);
n = 0;
for ki = 1:nKs
    for ci = 1:nCs
        for ji = 1:nCases
            n = n + 1;
            in = Simulink.SimulationInput(mdl);
            in = in.setVariable('road', cases(ji).road);
            in = in.setVariable('Ks', KsGrid(ki));
            in = in.setVariable('Cs', CsGrid(ci));
            in = in.setModelParameter('StopTime', num2str(cases(ji).duration));
            inputs(n) = in;
            flatKi(n) = ki;
            flatCi(n) = ci;
            flatJi(n) = ji;
        end
    end
end

% --- Run: parsim if available, serial sim() loop as fallback ---------
tSweep = tic;
try
    simOuts = parsim(inputs, 'ShowProgress', 'on', 'ShowSimulationManager', 'off');
    fprintf('Ran sweep with parsim.\n');
catch ME
    warning('tuneSuspension:parsimFailed', ...
        'parsim unavailable/failed (%s); falling back to serial sim().', ME.message);
    simOuts = Simulink.SimulationOutput.empty(0, 1);
    for n = 1:nTotal
        simOuts(n) = sim(inputs(n)); %#ok<AGROW>
    end
end
fprintf('Sweep took %.1f s.\n', toc(tSweep));

% --- Score every run, accumulate into [nKs x nCs x nCases] grids -----
rmsGrid = zeros(nKs, nCs, nCases);
travelGrid = zeros(nKs, nCs, nCases);
deflGrid = zeros(nKs, nCs, nCases);
passGrid = false(nKs, nCs, nCases);
scoreGrid = zeros(nKs, nCs, nCases);

for n = 1:nTotal
    r = scoreSuspension(simOuts(n), cases(flatJi(n)).name);
    rmsGrid(flatKi(n), flatCi(n), flatJi(n)) = r.rms_accel;
    travelGrid(flatKi(n), flatCi(n), flatJi(n)) = r.max_travel;
    deflGrid(flatKi(n), flatCi(n), flatJi(n)) = r.max_tireDefl;
    passGrid(flatKi(n), flatCi(n), flatJi(n)) = r.pass;
    scoreGrid(flatKi(n), flatCi(n), flatJi(n)) = r.score;
end

meanRMS = mean(rmsGrid, 3);
meanScore = mean(scoreGrid, 3);
worstTravel = max(travelGrid, [], 3);
worstDefl = max(deflGrid, [], 3);
allPass = all(passGrid, 3);

% --- Build results table -----------------------------------------------
[KsMesh, CsMesh] = ndgrid(KsGrid, CsGrid);
tuningTable = table(KsMesh(:), CsMesh(:), meanRMS(:), meanScore(:), ...
    worstTravel(:), worstDefl(:), allPass(:), ...
    'VariableNames', {'Ks', 'Cs', 'meanRMSAccel', 'meanScore', ...
    'worstTravel', 'worstTireDefl', 'allCasesPass'});

resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~isfolder(resultsDir); mkdir(resultsDir); end
writetable(tuningTable, fullfile(resultsDir, 'tuning_table.csv'));

% --- Select best design ------------------------------------------------
passIdx = find(tuningTable.allCasesPass);
if ~isempty(passIdx)
    [~, best] = min(tuningTable.meanRMSAccel(passIdx));
    bestRow = passIdx(best);
    fprintf('\n%d / %d designs meet constraints on every road case.\n', numel(passIdx), height(tuningTable));
else
    warning('tuneSuspension:noFeasibleDesign', ...
        'No (Ks,Cs) in the sweep range meets constraints on every road case; selecting the least-infeasible design by penalized score. Consider widening the sweep range.');
    [~, bestRow] = min(tuningTable.meanScore);
end
bestKs = tuningTable.Ks(bestRow);
bestCs = tuningTable.Cs(bestRow);

fprintf('Selected design: Ks=%g N/m, Cs=%g Ns/m\n', bestKs, bestCs);
fprintf('  mean RMS accel = %.4f m/s^2\n', tuningTable.meanRMSAccel(bestRow));
fprintf('  worst-case travel = %.4f m (limit 0.08)\n', tuningTable.worstTravel(bestRow));
fprintf('  worst-case tire deflection = %.4f m (limit 0.02)\n', tuningTable.worstTireDefl(bestRow));

% --- Baseline for comparison (Ks=20000, Cs=750, present in the grid) -
baseRow = find(tuningTable.Ks == 20000 & tuningTable.Cs == 750, 1);

% --- Plot 1: heatmap of mean comfort score with feasible region -------
f1 = figure('Visible', 'off', 'Position', [100 100 700 550]);
imagesc(CsGrid, KsGrid, meanRMS); set(gca, 'YDir', 'normal'); colorbar;
xlabel('Cs (Ns/m)'); ylabel('Ks (N/m)');
title('Mean RMS body acceleration across all road cases (m/s^2)');
hold on;
[cRow, cCol] = find(allPass');
if ~isempty(cRow)
    plot(CsGrid(cRow), KsGrid(cCol), 'go', 'MarkerSize', 6, 'LineWidth', 1.2, ...
        'DisplayName', 'meets constraints');
end
plot(bestCs, bestKs, 'rp', 'MarkerSize', 16, 'MarkerFaceColor', 'r', 'DisplayName', 'selected');
plot(750, 20000, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'w', 'DisplayName', 'baseline');
legend('Location', 'bestoutside');
saveas(f1, fullfile(resultsDir, 'tuning_heatmap.png'));
close(f1);

% --- Plot 2: before/after comparison per road case ---------------------
baseSims = cell(nCases, 1);
tunedSims = cell(nCases, 1);
for ji = 1:nCases
    baseSims{ji} = scoreSuspension(simulateRoadCase(cases(ji), 20000, 750), cases(ji).name);
    tunedSims{ji} = scoreSuspension(simulateRoadCase(cases(ji), bestKs, bestCs), cases(ji).name);
end
baseRMS = cellfun(@(s) s.rms_accel, baseSims);
tunedRMS = cellfun(@(s) s.rms_accel, tunedSims);
baseTravel = cellfun(@(s) s.max_travel, baseSims);
tunedTravel = cellfun(@(s) s.max_travel, tunedSims);

f2 = figure('Visible', 'off', 'Position', [100 100 1000 450]);
subplot(1,2,1);
bar([baseRMS, tunedRMS]);
set(gca, 'XTickLabel', {cases.name}); xtickangle(30);
ylabel('RMS body accel (m/s^2)'); title('Comfort: baseline vs tuned');
legend({'baseline (20000,750)', sprintf('tuned (%g,%g)', bestKs, bestCs)}, 'Location', 'best');
grid on;

subplot(1,2,2);
bar([baseTravel, tunedTravel]); hold on;
yline(0.08, 'r--', 'travel limit');
set(gca, 'XTickLabel', {cases.name}); xtickangle(30);
ylabel('Max suspension travel (m)'); title('Packaging: baseline vs tuned');
legend({'baseline', 'tuned', 'limit'}, 'Location', 'best');
grid on;

saveas(f2, fullfile(resultsDir, 'tuning_comparison.png'));
close(f2);

fprintf('Saved results/tuning_table.csv, results/tuning_heatmap.png, results/tuning_comparison.png\n');

end
