% initParams.m
% Baseline workspace setup so model.slx can be opened and run standalone
% (e.g. by pressing Play in Simulink) without any test harness.
%
% Suspension parameters (Ks, Cs) are wired into model.slx as named
% variables via tools/configureModelIO_step1.m -- baseline values below.
Ks = 20000;   % suspension spring stiffness [N/m]
Cs = 750;     % suspension damping coefficient [Ns/m]

% Default road input for standalone runs: flat road for 0.5 s, then a
% 50 mm half-sine speed bump lasting 0.3 s, then flat again.
Ts = 1e-3;
t  = (0:Ts:3)';
z  = zeros(size(t));
bumpStart = 0.5; bumpDur = 0.3; bumpHeight = 0.05;
inBump = t >= bumpStart & t <= bumpStart+bumpDur;
z(inBump) = bumpHeight * sin(pi*(t(inBump)-bumpStart)/bumpDur);
road = timeseries(z, t, 'Name', 'road');

clear Ts t z bumpStart bumpDur bumpHeight inBump
