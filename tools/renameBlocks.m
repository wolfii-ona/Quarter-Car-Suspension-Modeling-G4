% renameBlocks.m
% Cosmetic-only rename of blocks in model.slx to align naming with the
% classmate's samplemodel.slx where an equivalent block exists, and to use
% descriptive names where it doesn't. No wiring, parameters, or logic are
% changed.
%
% NOTE: the four root Outport blocks (x_road, x_wheel, x_body, az_body) are
% deliberately NOT renamed -- tools/getLoggedSignal.m matches signals by
% those Outport block names, so renaming them would break the whole
% analysis pipeline.

mdl = 'model';
projRoot = fileparts(fileparts(mfilename('fullpath')));
cd(projRoot);
close_system(mdl, 0);
load_system(fullfile(projRoot, 'model.slx'));

% old name -> new name
map = {
    'Brick Solid1',              'Body'                      % matches samplemodel
    'Cylindrical Solid',         'Wheel'                     % matches samplemodel
    'Prismatic Joint',           'Body Joint'                % matches samplemodel
    'Prismatic Joint1',          'Wheel joint'               % matches samplemodel
    'Prismatic Joint2',          'Road input'                % matches samplemodel
    'Brick Solid',               'Road plate'                % no counterpart (samplemodel has no road mass)
    'Spring and Damper Force',   'Suspension Spring Damper'  % no counterpart (samplemodel builds these into the joints)
    'Spring and Damper Force1',  'Tire Spring Damper'        % no counterpart
};

for i = 1:size(map, 1)
    oldPath = [mdl '/' map{i,1}];
    set_param(oldPath, 'Name', map{i,2});
    fprintf('renamed  %-26s -> %s\n', map{i,1}, map{i,2});
end

save_system(mdl);
fprintf('\nSaved model.slx.\n');

fprintf('\n--- block list after rename ---\n');
blocks = find_system(mdl, 'Type', 'Block');
for i = 1:numel(blocks)
    fprintf('  %s\n', strrep(blocks{i}, sprintf(char(10)), ' '));
end
