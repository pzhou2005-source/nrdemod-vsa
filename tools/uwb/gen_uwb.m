function gen_uwb(outRoot)
%GEN_UWB HRP UWB waveforms: 802.15.4a / 4z BPRF / 4z HPRF with lrwpanWaveformGenerator
%   (Communications Toolbox) and 802.15.4ab EHPRF with the helperEHPRF* supporting
%   files of the "HRP UWB IEEE 802.15.4ab Waveform Generation" example.
%   Output: <outRoot>/uwb/<name>.txt + .para2 (INPUT names follow the 89600 802154HrpUwb para2).
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'wavegen_common'));
if nargin < 1, outRoot = wg_root(); end
outDir = fullfile(outRoot, 'uwb');

% name, lrwpanHRPConfig name-value pairs
cases = {
    'UWB_4z_BPRF_ch9_CI9_SP0_PSDU127',  {'Mode', 'BPRF', 'Channel', 9, 'CodeIndex', 9,  'PHRDataRate', 0.85, 'STSPacketConfiguration', 0, 'PSDULength', 127}
    'UWB_4z_BPRF_ch5_CI10_SP1_PSDU127', {'Mode', 'BPRF', 'Channel', 5, 'CodeIndex', 10, 'PHRDataRate', 6.81, 'STSPacketConfiguration', 1, 'PSDULength', 127}
    'UWB_4z_BPRF_ch9_CI11_SP3',         {'Mode', 'BPRF', 'Channel', 9, 'CodeIndex', 11, 'STSPacketConfiguration', 3, 'PSDULength', 0}
    'UWB_4z_HPRF_ch9_CI25_249M6_PSDU127', {'Mode', 'HPRF', 'Channel', 9, 'MeanPRF', 249.6, 'CodeIndex', 25, 'STSPacketConfiguration', 1, 'PSDULength', 127}
    'UWB_4z_HPRF_ch5_CI27_124M8_PSDU100', {'Mode', 'HPRF', 'Channel', 5, 'MeanPRF', 124.8, 'CodeIndex', 27, 'PreambleDuration', 32, 'SFDNumber', 1, ...
                                           'ConstraintLength', 7, 'STSPacketConfiguration', 1, 'NumSTSSegments', 2, 'PSDULength', 100}
    'UWB_4a_ch9_CI3_PRF15M6_6M81_PSDU100', {'Mode', '802.15.4a', 'Channel', 9, 'MeanPRF', 15.6, 'DataRate', 6.81, 'CodeIndex', 3, 'PSDULength', 100}
    };
% name, helperEHPRFConfig name-value pairs (802.15.4ab draft)
casesAb = {
    'UWB_4ab_EHPRF_ch9_62M4_BCC_PSDU256',   {'Channel', 9, 'BitRate', 62.4,  'DynamicPHR', true, 'Coding', 'BCC',  'PSDULength', 256}
    'UWB_4ab_EHPRF_ch9_124M8_LDPC_PSDU1024', {'Channel', 9, 'BitRate', 124.8, 'DynamicPHR', true, 'Coding', 'LDPC', 'PSDULength', 1024}
    'UWB_4ab_EHPRF_ch5_7M8_LDPC_PSDU127',    {'Channel', 5, 'BitRate', 7.8,   'DynamicPHR', true, 'Coding', 'LDPC', 'PSDULength', 127}
    };

rng(2026);
fprintf('UWB -> %s\n', outDir);
for k = 1:size(cases, 1)
    cfg = lrwpanHRPConfig(cases{k, 2}{:});
    psdu = randi([0 1], max(8 * cfg.PSDULength, 8), 1);  % SP3 (no PSDU) still needs a data vector
    wave = lrwpanWaveformGenerator(psdu, cfg);
    write_one(outDir, cases{k, 1}, wave, psdu, cfg, 'lrwpanWaveformGenerator');
end

wg_require_example('helperEHPRFWaveformGenerator', 'comm/IEEE802154abWaveformGenerationHRPUWBExample');
for k = 1:size(casesAb, 1)
    cfg = helperEHPRFConfig(casesAb{k, 2}{:});
    psdu = randi([0 1], 8 * cfg.PSDULength, 1);
    wave = helperEHPRFWaveformGenerator(psdu, cfg);
    write_one(outDir, casesAb{k, 1}, wave, psdu, cfg, 'helperEHPRFWaveformGenerator');
end
end

function write_one(outDir, stem, wave, psdu, cfg, generator)
fs = cfg.SampleRate;
pad = zeros(round(2e-6 * fs), 1);
mode = char(cfg.Mode);
switch mode
    case '802.15.4a', phyMode = 0; deltaL = 16 + 48 * (cfg.PreambleMeanPRF < 5);
    case 'BPRF',      phyMode = 1; deltaL = 4;
    case 'HPRF',      phyMode = 2; deltaL = 4;
    otherwise,        phyMode = 3; deltaL = 4;  % EHPRF (802.15.4ab draft)
end
params = {
    'CodeIndex',        'Int',    cfg.CodeIndex
    'DeltaLength',      'Int',    deltaL
    'PhyMode',          'Int',    phyMode
    'SfdIndex',         'Int',    cfg.SFDNumber
    'StsPacketConfig',  'Int',    cfg.STSPacketConfiguration
    'SyncLength',       'Int',    cfg.PreambleDuration
    'Mode',             'String', mode
    'Channel',          'Int',    cfg.Channel
    'NumSTSSegments',   'Int',    cfg.NumSTSSegments
    'STSSegmentLength', 'Int',    cfg.STSSegmentLength
    'SamplesPerPulse',  'Int',    cfg.SamplesPerPulse
    'PSDULength',       'Int',    cfg.PSDULength
    };
if phyMode == 3
    params = [params; {
        'BitRate',      'Double', cfg.BitRate
        'DynamicPHR',   'Bool',   cfg.DynamicPHR
        'Coding',       'String', cfg.Coding
        }];
else
    params = [params; {
        'PhrDataRateMode',  'Int',    cfg.PHRDataRate > 1
        'MeanPRF',          'Double', cfg.MeanPRF
        'DataRate',         'Double', cfg.DataRate
        'PHRDataRate',      'Double', cfg.PHRDataRate
        'ConstraintLength', 'Int',    cfg.ConstraintLength
        'PreambleMeanPRF',  'Double', cfg.PreambleMeanPRF
        }];
end
params = [params; {'PayloadHex', 'String', wg_bits2hex(psdu(1:8 * cfg.PSDULength))}];
wg_write(outDir, stem, [pad; wave(:); pad], fs, uwb_fc(cfg.Channel), '802154HrpUwb', generator, params);
end

function fc = uwb_fc(ch)
% IEEE 802.15.4 HRP UWB channel centre frequencies, index = channel + 1
f = [499.2 3494.4 3993.6 4492.8 3993.6 6489.6 6988.8 6489.6 7488.0 7987.2 8486.4 7987.2 8985.6 9484.8 9984.0 9484.8];
fc = f(ch + 1) * 1e6;
end
