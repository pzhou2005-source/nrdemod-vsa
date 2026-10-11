function generate_all_waveforms(outRoot, subset)
%GENERATE_ALL_WAVEFORMS Run every protocol generator under tools/<protocol>/gen_<protocol>.m.
%   generate_all_waveforms                      -> all protocols into captures/generated
%   generate_all_waveforms([], {'nr5g','ble'})  -> a subset
%   Command line: matlab -batch "cd tools; generate_all_waveforms"
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'wavegen_common'));
if nargin < 1 || isempty(outRoot), outRoot = wg_root(); end
protocols = {'nr5g', 'uwb', 'bt_hdt', 'ble', 'bt_bredr', 'wifi', 'sparklink'};
if nargin >= 2, protocols = cellstr(subset); end

failed = {};
for k = 1:numel(protocols)
    p = protocols{k};
    addpath(fullfile(here, p));
    try
        feval(['gen_' p], outRoot);
    catch err
        fprintf(2, '%s FAILED: %s\n', p, err.message);
        failed{end + 1} = p; %#ok<AGROW>
    end
end
fprintf('\n%d/%d protocols generated into %s\n', numel(protocols) - numel(failed), numel(protocols), outRoot);
if ~isempty(failed)
    error('generate_all_waveforms:failed', 'failed: %s', strjoin(failed, ', '));
end
end
