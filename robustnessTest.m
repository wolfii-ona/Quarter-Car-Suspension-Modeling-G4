function [robSummary, robTable] = robustnessTest(Ks0, Cs0, nTrials)
% ROBUSTNESSTEST Monte Carlo robustness check: apply +/-10% independent
% uniform variation to suspension stiffness (Ks) and damping (Cs) around
% a nominal design, rerun the full road suite for each trial, and
% report worst-case metrics + pass rate.
%
% [robSummary, robTable] = robustnessTest(Ks0, Cs0, nTrials)
%
%   Ks0, Cs0 - nominal design point. Defaults to the tuned design
%              selected by tuneSuspension.m (20000, 2250) if omitted.
%   nTrials  - number of Monte Carlo trials (default 20, per project
%              spec).
%
% Saves results/robustness_table.csv and results/robustness_summary.png.
% Uses a fixed RNG seed for reproducibility.

addpath(fullfile(fileparts(mfilename('fullpath')), 'tools'));

if nargin < 1 || isempty(Ks0); Ks0 = 20000; end
if nargin < 2 || isempty(Cs0); Cs0 = 2250; end
if nargin < 3 || isempty(nTrials); nTrials = 20; end

mdl = 'model';
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

rng(7); % fixed seed for reproducible trials
tolKs = Ks0 * 0.10 * (2*rand(nTrials, 1) - 1); % uniform +/-10%
tolCs = Cs0 * 0.10 * (2*rand(nTrials, 1) - 1);
KsTrials = Ks0 + tolKs;
CsTrials = Cs0 + tolCs;

cases = roadSuite();
nCases = numel(cases);

fprintf('Running %d Monte Carlo trials (+/-10%% on Ks=%g, Cs=%g) x %d road cases = %d simulations...\n', ...
    nTrials, Ks0, Cs0, nCases, nTrials*nCases);

% --- Build flattened SimulationInput array ---------------------------
nTotal = nTrials * nCases;
inputs = Simulink.SimulationInput.empty(0, 1);
flatTi = zeros(nTotal, 1);
flatJi = zeros(nTotal, 1);
n = 0;
for ti = 1:nTrials
    for ji = 1:nCases
        n = n + 1;
        in = Simulink.SimulationInput(mdl);
        in = in.setVariable('road', cases(ji).road);
        in = in.setVariable('Ks', KsTrials(ti));
        in = in.setVariable('Cs', CsTrials(ti));
        in = in.setModelParameter('StopTime', num2str(cases(ji).duration));
        inputs(n) = in;
        flatTi(n) = ti;
        flatJi(n) = ji;
    end
end

try
    simOuts = parsim(inputs, 'ShowProgress', 'on', 'ShowSimulationManager', 'off');
    fprintf('Ran robustness trials with parsim.\n');
catch ME
    warning('robustnessTest:parsimFailed', ...
        'parsim unavailable/failed (%s); falling back to serial sim().', ME.message);
    simOuts = Simulink.SimulationOutput.empty(0, 1);
    for n = 1:nTotal
        simOuts(n) = sim(inputs(n)); %#ok<AGROW>
    end
end

% --- Score every run, accumulate into [nTrials x nCases] grids -------
rmsGrid = zeros(nTrials, nCases);
travelGrid = zeros(nTrials, nCases);
deflGrid = zeros(nTrials, nCases);
passGrid = false(nTrials, nCases);

for n = 1:nTotal
    r = scoreSuspension(simOuts(n), cases(flatJi(n)).name);
    rmsGrid(flatTi(n), flatJi(n)) = r.rms_accel;
    travelGrid(flatTi(n), flatJi(n)) = r.max_travel;
    deflGrid(flatTi(n), flatJi(n)) = r.max_tireDefl;
    passGrid(flatTi(n), flatJi(n)) = r.pass;
end

trialWorstRMS = max(rmsGrid, [], 2);
trialWorstTravel = max(travelGrid, [], 2);
trialWorstDefl = max(deflGrid, [], 2);
trialAllPass = all(passGrid, 2);

robTable = table((1:nTrials)', KsTrials, CsTrials, trialWorstRMS, ...
    trialWorstTravel, trialWorstDefl, trialAllPass, ...
    'VariableNames', {'trial', 'Ks', 'Cs', 'worstRMSAccel', ...
    'worstTravel', 'worstTireDefl', 'allCasesPass'});

resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~isfolder(resultsDir); mkdir(resultsDir); end
writetable(robTable, fullfile(resultsDir, 'robustness_table.csv'));

passRate = mean(trialAllPass);
robSummary = struct( ...
    'Ks0', Ks0, 'Cs0', Cs0, 'nTrials', nTrials, ...
    'passRate', passRate, ...
    'worstCaseRMSAccel', max(trialWorstRMS), ...
    'worstCaseTravel', max(trialWorstTravel), ...
    'worstCaseTireDefl', max(trialWorstDefl), ...
    'travel_limit', 0.08, ...
    'tireDefl_limit', 0.02);

fprintf('\nRobustness summary (nominal Ks=%g, Cs=%g, +/-10%%, %d trials):\n', Ks0, Cs0, nTrials);
fprintf('  Pass rate (all 5 road cases pass): %d / %d trials = %.0f%%\n', ...
    sum(trialAllPass), nTrials, 100*passRate);
fprintf('  Worst-case RMS accel across all trials: %.4f m/s^2\n', robSummary.worstCaseRMSAccel);
fprintf('  Worst-case suspension travel: %.4f m (limit %.2f)\n', robSummary.worstCaseTravel, robSummary.travel_limit);
fprintf('  Worst-case tire deflection: %.4f m (limit %.2f)\n', robSummary.worstCaseTireDefl, robSummary.tireDefl_limit);

% --- Plot: trial scatter + histograms ---------------------------------
f = figure('Visible', 'off', 'Position', [100 100 1000 700]);

subplot(2,2,1);
scatter(KsTrials, CsTrials, 40, trialAllPass, 'filled');
hold on; plot(Ks0, Cs0, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'y');
xlabel('Ks (N/m)'); ylabel('Cs (Ns/m)');
title('Trials (yellow star = nominal design)'); colormap(gca, [0.85 0.2 0.2; 0.2 0.7 0.2]);
cb = colorbar; cb.Ticks = [0.25 0.75]; cb.TickLabels = {'fail','pass'};
grid on;

subplot(2,2,2);
histogram(trialWorstRMS, 8);
xlabel('Worst-case RMS accel per trial (m/s^2)'); ylabel('Trial count');
title('Comfort spread across trials'); grid on;

subplot(2,2,3);
histogram(trialWorstTravel, 8); hold on;
xline(0.08, 'r--', 'limit');
xlabel('Worst-case suspension travel per trial (m)'); ylabel('Trial count');
title('Packaging spread across trials'); grid on;

subplot(2,2,4);
histogram(trialWorstDefl, 8); hold on;
xline(0.02, 'r--', 'limit');
xlabel('Worst-case tire deflection per trial (m)'); ylabel('Trial count');
title('Road-holding spread across trials'); grid on;

sgtitle(sprintf('Robustness: +/-10%% Ks/Cs Monte Carlo (n=%d), %.0f%% pass rate', nTrials, 100*passRate));
saveas(f, fullfile(resultsDir, 'robustness_summary.png'));
close(f);
fprintf('Saved results/robustness_table.csv, results/robustness_summary.png\n');

end
