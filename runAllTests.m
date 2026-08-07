function [summaryTable, allResults] = runAllTests(Ks, Cs)
% RUNALLTESTS Run the full road test suite against model.slx at given
% suspension parameters, score every case, print pass/fail, and save a
% summary figure.
%
% [summaryTable, allResults] = runAllTests(Ks, Cs)
%
%   Ks, Cs  - optional suspension stiffness [N/m] / damping [Ns/m].
%             Default to baseline (20000, 750) if omitted.
%
% Returns summaryTable (one row per road case) and allResults (struct
% array of the full scoreSuspension() output per case). Saves
% results/summary.png.

addpath(fullfile(fileparts(mfilename('fullpath')), 'tools'));

if nargin < 1 || isempty(Ks); Ks = 20000; end
if nargin < 2 || isempty(Cs); Cs = 750; end

mdl = 'model';
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

cases = roadSuite();
nCases = numel(cases);
allResults = struct([]);
allSignals = cell(nCases, 1);

fprintf('Running %d road cases at Ks=%g N/m, Cs=%g Ns/m...\n', nCases, Ks, Cs);
for i = 1:nCases
    simOut = simulateRoadCase(cases(i), Ks, Cs);
    r = scoreSuspension(simOut, cases(i).name);
    if isempty(allResults)
        allResults = r;
    else
        allResults(end+1) = r; %#ok<AGROW>
    end
    allSignals{i} = struct( ...
        'x_road', getLoggedSignal(simOut, 'x_road'), ...
        'x_wheel', getLoggedSignal(simOut, 'x_wheel'), ...
        'x_body', getLoggedSignal(simOut, 'x_body'), ...
        'az_body', getLoggedSignal(simOut, 'az_body'));

    status = 'PASS';
    if ~r.pass; status = 'FAIL'; end
    fprintf('  [%-4s] %-10s  RMS accel=%.3f m/s^2  max travel=%.4f m (lim %.2f)  max tire defl=%.4f m (lim %.2f)  score=%.3f\n', ...
        status, r.roadName, r.rms_accel, r.max_travel, r.travel_limit, ...
        r.max_tireDefl, r.tireDefl_limit, r.score);
end

summaryTable = struct2table(rmfield(allResults, {'pass_travel', 'pass_tireDefl'}));

nPass = sum([allResults.pass]);
fprintf('\n%d / %d road cases passed constraints.\n', nPass, nCases);

resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~isfolder(resultsDir); mkdir(resultsDir); end

f = figure('Visible', 'off', 'Position', [100 100 1200 800]);
for i = 1:nCases
    subplot(nCases, 2, 2*i-1);
    s = allSignals{i};
    plot(s.x_road.Time, s.x_road.Data, 'DisplayName', 'road'); hold on;
    plot(s.x_wheel.Time, s.x_wheel.Data, 'DisplayName', 'wheel');
    plot(s.x_body.Time, s.x_body.Data, 'DisplayName', 'body');
    ylabel(cases(i).name, 'FontWeight', 'bold');
    if i == 1; title('Displacements (m)'); legend('show', 'Location', 'eastoutside'); end
    if i == nCases; xlabel('Time (s)'); end
    grid on;

    subplot(nCases, 2, 2*i);
    plot(s.az_body.Time, s.az_body.Data, 'Color', [0.85 0.2 0.2]);
    if i == 1; title('Body vertical acceleration (m/s^2)'); end
    if i == nCases; xlabel('Time (s)'); end
    grid on;
end
sgtitle(sprintf('runAllTests summary -- Ks=%g N/m, Cs=%g Ns/m (%d/%d pass)', ...
    Ks, Cs, nPass, nCases));
saveas(f, fullfile(resultsDir, 'summary.png'));
close(f);
fprintf('Saved results/summary.png\n');

end
