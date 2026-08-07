function cases = roadSuite()
% ROADSUITE Generate the named road test suite for the quarter-car
% suspension model.
%
% Returns a struct array, one entry per road case, with fields:
%   name         - short identifier, e.g. 'SpeedBump'
%   description  - one-line note on how the profile was made
%   road         - timeseries of road vertical displacement [m], the
%                  variable the model's From Workspace block expects
%   duration     - total simulated time [s] for this case
%   metricStart  - time [s] after which metrics are computed. Every
%                  case has a flat 1 s lead-in so the model settles from
%                  its initial free-fall transient (no static preload
%                  IC is set) before the actual road disturbance
%                  starts; metrics exclude that settling period so they
%                  reflect the road response only, not simulation
%                  startup artifacts.
%
% All profiles use a common sample time and start with 1 s of flat
% (zero) road so every case begins from the same settled condition.

Ts = 1e-3;
leadIn = 1.0;

cases = struct('name', {}, 'description', {}, 'road', {}, ...
    'duration', {}, 'metricStart', {});

% --- 1. Speed bump: smooth half-sine hump -------------------------
dur = 5.0;
t = (0:Ts:dur)';
z = zeros(size(t));
z = z + halfSineBump(t, leadIn, 0.3, 0.05);
cases(end+1) = makeCase('SpeedBump', ...
    'Half-sine hump, 50 mm peak over 0.3 s.', t, z, dur, leadIn);

% --- 2. Pothole: smooth dip ----------------------------------------
dur = 5.0;
t = (0:Ts:dur)';
z = zeros(size(t));
z = z + halfSineBump(t, leadIn, 0.3, -0.05);
cases(end+1) = makeCase('Pothole', ...
    'Half-sine dip, -50 mm peak over 0.3 s.', t, z, dur, leadIn);

% --- 3. Rough road: band-limited noise ------------------------------
dur = 9.0;
t = (0:Ts:dur)';
z = zeros(size(t));
rng(42); % fixed seed for repeatable comparisons across designs/runs
noiseIdx = t >= leadIn;
nSamp = sum(noiseIdx);
whiteNoise = randn(nSamp, 1);
[bf, af] = butter(4, 20 / (0.5 / Ts)); % 20 Hz low-pass, Fs = 1/Ts
filtered = filtfilt(bf, af, whiteNoise);
filtered = filtered / rms(filtered) * 0.008; % scale to 8 mm RMS
% 0.2 s smooth fade-in so the noise doesn't start with a velocity jump
fadeN = round(0.2 / Ts);
fadeN = min(fadeN, nSamp);
env = ones(nSamp, 1);
env(1:fadeN) = (1 - cos(pi * (0:fadeN-1)' / fadeN)) / 2;
z(noiseIdx) = filtered .* env;
cases(end+1) = makeCase('RoughRoad', ...
    '20 Hz low-pass filtered white noise, 8 mm RMS, seed=42.', t, z, dur, leadIn);

% --- 4. Washboard: sinusoidal corrugation ---------------------------
dur = 6.0;
t = (0:Ts:dur)';
z = zeros(size(t));
freq = 4.0; ampl = 0.01; sinDur = 3.0;
sinIdx = t >= leadIn & t <= leadIn + sinDur;
tau = t(sinIdx) - leadIn;
env = sin(pi * tau / sinDur).^2; % smooth ramp in/out over the burst
z(sinIdx) = ampl * sin(2*pi*freq*tau) .* env;
cases(end+1) = makeCase('Washboard', ...
    '4 Hz sinusoidal corrugation, 10 mm amplitude, 3 s with raised-cosine envelope.', ...
    t, z, dur, leadIn);

% --- 5. Two bumps: repeatability / transient recovery ---------------
dur = 6.0;
t = (0:Ts:dur)';
z = zeros(size(t));
z = z + halfSineBump(t, leadIn, 0.3, 0.05);
z = z + halfSineBump(t, leadIn + 1.5, 0.3, 0.05);
cases(end+1) = makeCase('TwoBumps', ...
    'Two 50 mm half-sine bumps, 1.5 s apart, tests transient recovery.', ...
    t, z, dur, leadIn);

end

function c = makeCase(name, desc, t, z, dur, metricStart)
c.name = name;
c.description = desc;
c.road = timeseries(z, t, 'Name', 'road');
c.duration = dur;
c.metricStart = metricStart;
end

function z = halfSineBump(t, startTime, bumpDur, height)
z = zeros(size(t));
inBump = t >= startTime & t <= startTime + bumpDur;
z(inBump) = height * sin(pi * (t(inBump) - startTime) / bumpDur);
end
