function wg_require_example(helperName, exampleId)
%WG_REQUIRE_EXAMPLE Put the supporting files of a MathWorks example on the path.
%   e.g. wg_require_example('helperBLEHDTWaveform', ...
%            'bluetooth/EndtoEndSimulationOfBluetoothHDTWaveformsExample')
if exist(helperName, 'file')
    return
end
here = pwd;
restore = onCleanup(@() cd(here));
try
    openExample(exampleId);  % copies the example folder and cd's into it
    addpath(pwd);
catch err
    error('wg:example', ['%s not found and openExample(''%s'') failed (%s).\n' ...
        'Open the example once in MATLAB and add its folder to the path.'], ...
        helperName, exampleId, err.message);
end
assert(exist(helperName, 'file') > 0, '%s still not on the path after openExample', helperName);
end
