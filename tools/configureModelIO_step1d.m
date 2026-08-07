% Step 1d: physics/solver fixes found by actually running the model
% (these were applied as one-off commands during debugging; recorded
% here afterward so the full edit history is reviewable as scripts,
% same as steps 1a-1c).
%
% 1) Road joint (Prismatic Joint2) needs TorqueActuationMode=ComputedTorque
%    so Simscape can compute the reaction force for its prescribed
%    (InputMotion) motion -- required for the solver's DOF/actuation
%    balance check.
% 2) The road's Simulink-PS Converter1 needs input filtering (order 2)
%    so the solver has the velocity/acceleration derivatives it needs
%    to drive the prescribed-motion joint.
% 3) Brick Solid1 (body) and Cylindrical Solid (wheel) had
%    BasedOnType='Density', so Simscape computed their mass from
%    placeholder 1x1x1 m geometry (1000 kg / ~3142 kg) instead of using
%    the intended Mass=350 / Mass=20 literals. Switched to
%    BasedOnType='Mass'.
% 4) Both spring/damper blocks had NaturalLength=0, but the
%    RigidTransform blocks (used for 3-D visualization spacing) place
%    the bodies 0.5 m apart at the zero-DOF assembly configuration --
%    so each spring was pre-compressed by 0.5 m before gravity even
%    acted. Set NaturalLength=0.5 on both to match the actual assembled
%    geometry, so gravity is the only source of static sag.

mdl = 'model';
projRoot = fileparts(fileparts(mfilename('fullpath')));
cd(projRoot);
close_system(mdl,0);
load_system(fullfile(projRoot,'model.slx'));

set_param('model/Prismatic Joint2','TorqueActuationMode','ComputedTorque');

set_param('model/Simulink-PS Converter1','FilteringAndDerivatives','filter');
set_param('model/Simulink-PS Converter1','SimscapeFilterOrder','2');

set_param('model/Brick Solid1','BasedOnType','Mass');
set_param('model/Cylindrical Solid','BasedOnType','Mass');

set_param('model/Spring and Damper Force','NaturalLength','0.5');
set_param('model/Spring and Damper Force1','NaturalLength','0.5');

save_system(mdl);
fprintf('Saved model.slx with physics/solver fixes (step 1d).\n');
