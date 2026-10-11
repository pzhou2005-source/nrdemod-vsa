function gen_wifi(outRoot)
%GEN_WIFI Wi-Fi 6 (802.11ax HE SU) and Wi-Fi 7 (802.11be EHT MU, single user) packets with
%   wlanWaveformGenerator (WLAN Toolbox).
%   Output: <outRoot>/wifi/<name>.txt + .para2.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'wavegen_common'));
if nargin < 1, outRoot = wg_root(); end
outDir = fullfile(outRoot, 'wifi');

%        format  CBW       MCS  APEPLength  fc
cases = {
    'HE',  'CBW20',   0,  1000, 5.18e9
    'HE',  'CBW20',   7,  1000, 5.18e9
    'HE',  'CBW40',   9,  2000, 5.19e9
    'HE',  'CBW80',  11,  2000, 5.21e9
    'HE',  'CBW160', 11,  4000, 5.25e9
    'EHT', 'CBW20',  13,  1000, 5.955e9
    'EHT', 'CBW80',  12,  2000, 5.985e9
    'EHT', 'CBW160', 13,  4000, 6.025e9
    'EHT', 'CBW320', 13,  4000, 6.105e9
    };
gi = 0.8;   % us
ltf = 2;    % 2x HE/EHT-LTF

rng(2026);
fprintf('Wi-Fi 6/7 -> %s\n', outDir);
for k = 1:size(cases, 1)
    [fmt, cbw, mcs, apep, fc] = cases{k, :};
    if strcmp(fmt, 'HE')
        cfg = wlanHESUConfig('ChannelBandwidth', cbw, 'MCS', mcs, 'APEPLength', apep, ...
            'GuardInterval', gi, 'HELTFType', ltf);
        standard = '80211ax';
        fmtName = 'HE-SU';
    else
        cfg = wlanEHTMUConfig(cbw);
        cfg.User{1}.MCS = mcs;
        cfg.User{1}.APEPLength = apep;
        cfg.GuardInterval = gi;
        cfg.EHTLTFType = ltf;
        standard = '80211be';
        fmtName = 'EHT-MU';
    end
    if ismethod(cfg, 'psduLength')
        nPsdu = psduLength(cfg);
    else
        nPsdu = getPSDULength(cfg);
    end
    psdu = randi([0 1], 8 * nPsdu(1), 1);
    if strcmp(fmt, 'EHT')
        wave = wlanWaveformGenerator({psdu}, cfg);
    else
        wave = wlanWaveformGenerator(psdu, cfg);
    end
    fs = wlanSampleRate(cfg);
    pad = zeros(round(10e-6 * fs), 1);

    bwMHz = sscanf(cbw, 'CBW%d');
    stem = sprintf('WIFI_%s_%dMHz_MCS%d_%dB', strrep(fmtName, '-', ''), bwMHz, mcs, apep);
    params = {
        'Format',              'String', fmtName
        'ChannelBandwidth',    'Int',    bwMHz
        'MCS',                 'Int',    mcs
        'GuardInterval',       'Double', gi * 1e-6
        'LTFType',             'Int',    ltf
        'NumSpaceTimeStreams', 'Int',    1
        'SubcarrierSpacing',   'Double', 78125
        'APEPLength',          'Int',    apep
        'PSDULength',          'Int',    nPsdu(1)
        'PayloadHex',          'String', wg_bits2hex(psdu)
        };
    wg_write(outDir, stem, [pad; wave(:, 1); pad], fs, fc, standard, 'wlanWaveformGenerator', params);
end
end
