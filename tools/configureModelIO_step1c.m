% Step 1c: fix pre-existing model bug -- three joints were grounded to
% 'World Frame1', a second World Frame block NOT connected to the
% Solver Configuration's physical network (Simscape requires exactly
% one Solver Configuration per connected physical network). Rewire all
% three joints to the correctly-configured 'World Frame' block instead
% and remove the orphan.
%
% Note: deleting World Frame1 leaves stale line stubs on the ports that
% used to connect to it (Simscape doesn't clean these up automatically),
% so those must be explicitly deleted before the new lines are added.

mdl = 'model';
projRoot = fileparts(fileparts(mfilename('fullpath')));
cd(projRoot);
close_system(mdl,0);
load_system(fullfile(projRoot,'model.slx'));

delete_block('model/World Frame1');

joints = {'model/Prismatic Joint','model/Prismatic Joint1','model/Prismatic Joint2'};
for i = 1:numel(joints)
    lh = get_param(joints{i},'LineHandles');
    delete_line(lh.LConn(1));
end

phWorld = get_param('model/World Frame','PortHandles');
worldPort = phWorld.RConn(1);

for i = 1:numel(joints)
    ph = get_param(joints{i},'PortHandles');
    add_line('model', worldPort, ph.LConn(1), 'autorouting','on');
    fprintf('Rewired %s to World Frame\n', joints{i});
end

save_system(mdl);
fprintf('Saved model.slx with ground-frame fix.\n');
