function ts = getLoggedSignal(simOut, name)
% GETLOGGEDSIGNAL Extract a named root-output signal from a
% Simulink.SimulationOutput produced by model.slx.
%
% The model's root Outport blocks are named x_road/x_wheel/x_body/
% az_body and logged via yout (SaveFormat=Dataset). Dataset elements'
% .Name property is empty for unnamed signals feeding unnamed outports,
% so matching is done on the Outport block's name via BlockPath instead
% (verified reliable against this model).

yout = simOut.get('yout');
for i = 1:yout.numElements
    el = yout{i};
    bp = char(el.BlockPath.getBlock(1));
    parts = strsplit(bp, '/');
    if strcmp(parts{end}, name)
        ts = el.Values;
        return;
    end
end
error('getLoggedSignal:notFound', 'Signal "%s" not found in simulation output.', name);
end
