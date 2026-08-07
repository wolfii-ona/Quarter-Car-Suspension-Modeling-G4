% Step 1b: wire the 4 sensing signals out to root Outports via
% PS-Simulink Converter blocks. Run AFTER configureModelIO_step1.m.
% x_road  <- Prismatic Joint2 RConn(2)  (position, road plate)
% x_wheel <- Prismatic Joint1 RConn(2)  (position, wheel)
% x_body  <- Prismatic Joint  RConn(2)  (position, body)
% az_body <- Prismatic Joint  RConn(3)  (acceleration, body)

mdl = 'model';
projRoot = fileparts(fileparts(mfilename('fullpath')));
cd(projRoot);
close_system(mdl,0);
load_system(fullfile(projRoot,'model.slx'));

conv = sprintf('nesl_utility/PS-Simulink\nConverter');

specs = struct( ...
    'name',   {'x_road','x_wheel','x_body','az_body'}, ...
    'joint',  {'model/Prismatic Joint2','model/Prismatic Joint1','model/Prismatic Joint','model/Prismatic Joint'}, ...
    'rconn',  {2, 2, 2, 3}, ...
    'unit',   {'m','m','m','m/s^2'}, ...
    'ypos',   {-200, -140, -80, -20} ...
);

xConv = 950;
xOut  = 1050;

for i = 1:numel(specs)
    s = specs(i);
    convName = ['model/PS2S_' s.name];
    outName  = ['model/' s.name];

    % PS-Simulink Converter block
    hConv = add_block(conv, convName, 'MakeNameUnique', 'off', ...
        'Position', [xConv, s.ypos, xConv+40, s.ypos+20]);
    dp = fieldnames(get_param(hConv,'DialogParameters'));
    if any(strcmp(dp,'Unit'))
        set_param(hConv,'Unit', s.unit);
    end

    % Root Outport block
    hOut = add_block('simulink/Sinks/Out1', outName, 'MakeNameUnique','off', ...
        'Position', [xOut, s.ypos, xOut+30, s.ypos+20]);

    % Wire: joint sensing port -> converter input
    phJoint = get_param(s.joint,'PortHandles');
    srcPort = phJoint.RConn(s.rconn);
    phConv  = get_param(hConv,'PortHandles');
    add_line('model', srcPort, phConv.LConn(1), 'autorouting','on');

    % Wire: converter output -> Outport input
    phOut = get_param(hOut,'PortHandles');
    add_line('model', phConv.Outport(1), phOut.Inport(1), 'autorouting','on');

    fprintf('Wired %s (unit=%s) from %s RConn(%d)\n', s.name, s.unit, s.joint, s.rconn);
end

% Make Dataset output carry the Outport block names (x_road, x_wheel, ...)
set_param(mdl, 'SaveOutput', 'on');
set_param(mdl, 'OutputSaveName', 'yout');
set_param(mdl, 'SaveFormat', 'Dataset');
set_param(mdl, 'ReturnWorkspaceOutputs', 'on');

save_system(mdl);
fprintf('Saved model.slx with 4 wired output signals.\n');
