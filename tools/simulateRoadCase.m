function simOut = simulateRoadCase(roadCase, Ks, Cs)
% SIMULATEROADCASE Run model.slx for one road case with given
% suspension parameters, returning the Simulink.SimulationOutput.
%
% roadCase - struct from roadSuite() with fields .road (timeseries),
%            .name, .duration
% Ks, Cs   - optional suspension stiffness [N/m] / damping [Ns/m].
%            Default to baseline values (20000, 750) if omitted.

if nargin < 2 || isempty(Ks); Ks = 20000; end
if nargin < 3 || isempty(Cs); Cs = 750; end

mdl = 'model';
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

in = Simulink.SimulationInput(mdl);
in = in.setVariable('road', roadCase.road);
in = in.setVariable('Ks', Ks);
in = in.setVariable('Cs', Cs);
in = in.setModelParameter('StopTime', num2str(roadCase.duration));

simOut = sim(in);
end
