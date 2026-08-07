function results = scoreSuspension(simOut, roadName)
% SCORESUSPENSION Compute ride-comfort/packaging/road-holding metrics
% and a single scalar score from a simulation of one road case.
%
% results = scoreSuspension(simOut, roadName)
%
%   simOut   - Simulink.SimulationOutput from simulateRoadCase()
%   roadName - name of the road case simulated (must match a case
%              returned by roadSuite(), used to find the metric
%              analysis window)
%
% Metrics are computed only after roadCase.metricStart, and suspension
% travel / tire deflection are measured RELATIVE to their value at that
% start time -- i.e. as deviation from the static-gravity-loaded ride
% height, not absolute joint position. That's the physically meaningful
% quantity for packaging/clearance (how much MORE the suspension moves
% because of the road event), matching how "suspension travel" and
% "tire deflection" are normally specified for a loaded vehicle.
%
% Constraints (modeling assumptions, adjust as needed):
%   max suspension travel <= 0.08 m
%   max tire deflection   <= 0.02 m

addpath(fullfile(fileparts(mfilename('fullpath')), 'tools'));

TRAVEL_LIMIT = 0.08;
TIRE_DEFL_LIMIT = 0.02;
PENALTY_WEIGHT = 1000;

cases = roadSuite();
idx = find(strcmp({cases.name}, roadName), 1);
if isempty(idx)
    error('scoreSuspension:unknownRoad', 'Unknown road case "%s".', roadName);
end
metricStart = cases(idx).metricStart;

x_road  = getLoggedSignal(simOut, 'x_road');
x_wheel = getLoggedSignal(simOut, 'x_wheel');
x_body  = getLoggedSignal(simOut, 'x_body');
az_body = getLoggedSignal(simOut, 'az_body');

t = az_body.Time;
mask = t >= metricStart;

% Suspension travel / tire deflection, relative to their value at
% metricStart (i.e. deviation from settled static ride height).
travelSignal = x_body.Data - x_wheel.Data;
travelSignal = travelSignal - interpAt(t, travelSignal, metricStart);
deflSignal = x_wheel.Data - x_road.Data;
deflSignal = deflSignal - interpAt(t, deflSignal, metricStart);

accel = az_body.Data(mask);
travel = travelSignal(mask);
defl = deflSignal(mask);

rms_accel = rms(accel);
peak_accel = max(abs(accel));
max_travel = max(abs(travel));
max_tireDefl = max(abs(defl));

pass_travel = max_travel <= TRAVEL_LIMIT;
pass_tireDefl = max_tireDefl <= TIRE_DEFL_LIMIT;
pass = pass_travel && pass_tireDefl;

penalty = PENALTY_WEIGHT * max(0, max_travel - TRAVEL_LIMIT) + ...
          PENALTY_WEIGHT * max(0, max_tireDefl - TIRE_DEFL_LIMIT);
score = rms_accel + penalty;

results = struct( ...
    'roadName', roadName, ...
    'rms_accel', rms_accel, ...
    'peak_accel', peak_accel, ...
    'max_travel', max_travel, ...
    'max_tireDefl', max_tireDefl, ...
    'travel_limit', TRAVEL_LIMIT, ...
    'tireDefl_limit', TIRE_DEFL_LIMIT, ...
    'pass_travel', pass_travel, ...
    'pass_tireDefl', pass_tireDefl, ...
    'pass', pass, ...
    'score', score);
end

function v = interpAt(t, x, tq)
v = interp1(t, x, tq, 'linear', 'extrap');
end
