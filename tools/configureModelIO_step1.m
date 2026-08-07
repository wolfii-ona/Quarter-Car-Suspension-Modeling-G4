% Step 1a (inspection pass): parameterize Ks/Cs, enable sensing, remove
% dead orphan blocks. Run this, inspect printed port info, THEN wire
% outputs in step 1b. Kept in tools/ for reproducibility/audit trail.

mdl = 'model';
projRoot = fileparts(fileparts(mfilename('fullpath')));
cd(projRoot);
close_system(mdl,0); %#ok<*NASGU>
load_system(fullfile(projRoot,'model.slx'));

% Parameterize suspension spring/damper (Ks=20000, Cs=750) as named vars
set_param('model/Spring and Damper Force','SpringStiffness','Ks','DampingCoefficient','Cs');

% Enable required sensing
set_param('model/Prismatic Joint','SensePosition','on','SenseAcceleration','on'); % body
set_param('model/Prismatic Joint1','SensePosition','on'); % wheel
% Prismatic Joint2 (road) already has SensePosition=on

% Remove dead orphan blocks (unwired, unconfigured, contribute nothing)
delete_block('model/Transform Sensor');
delete_block('model/Simulink-PS Converter');

fprintf('=== Port counts after enabling sensing ===\n');
for b = {'model/Prismatic Joint','model/Prismatic Joint1','model/Prismatic Joint2'}
    bb = b{1};
    ph = get_param(bb,'PortHandles');
    fprintf('--- %s : Outport handles = %s\n', bb, mat2str(ph.Outport));
    for i=1:numel(ph.Outport)
        lbl = get_param(ph.Outport(i),'PropagatedSignals');
        try
            nm = get(ph.Outport(i),'Name');
        catch
            nm = '?';
        end
        fprintf('    port %d: PropagatedSignals=%s\n', i, lbl);
    end
end

save_system('model');
fprintf('Saved model.slx (step 1a changes only)\n');
