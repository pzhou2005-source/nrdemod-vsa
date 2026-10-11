function gen_bt_hdt(outRoot)
%GEN_BT_HDT Bluetooth LE High Data Throughput (HDT) bursts with helperBLEHDTWaveform from the
%   "Explore Link-Level Simulation of Bluetooth LE HDT PHY Waveforms" example (Bluetooth Toolbox, R2026a+).
%   Output: <outRoot>/bt_hdt/<name>.txt + .para2.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'wavegen_common'));
if nargin < 1, outRoot = wg_root(); end
outDir = fullfile(outRoot, 'bt_hdt');
wg_require_example('helperBLEHDTWaveform', 'bluetooth/EndtoEndSimulationOfBluetoothHDTWaveformsExample');

%        mode     packetFormat payloadBlockLength(symbols, format1 only)  octets
cases = {
    'HDT2',   'format0', [],  255
    'HDT3',   'format0', [],  255
    'HDT4',   'format0', [],  255
    'HDT6',   'format0', [],  255
    'HDT7.5', 'format0', [],  255
    'HDT6',   'format1', 128, 511
    'HDT7.5', 'format1', 128, 511
    };
sps = 8;
symbolRate = 2e6;
channelIndex = 0;
fc = wg_ble_rf_hz(channelIndex);

rng(2026);
fprintf('Bluetooth HDT -> %s\n', outDir);
for k = 1:size(cases, 1)
    [mode, pktFmt, blockLen, octets] = cases{k, :};
    [hdtParams, fecParams, filterParams] = helperBLEHDTInit(mode, pktFmt, blockLen, sps);
    data = randi([0 1], 8 * octets, 1);
    wave = helperBLEHDTWaveform(data, channelIndex, hdtParams, fecParams, filterParams);
    fs = symbolRate * sps;
    pad = zeros(round(10e-6 * fs), 1);

    stem = sprintf('BTHDT_%s_%s_%dB', strrep(mode, '.', 'p'), pktFmt, octets);
    if isempty(blockLen), blockLen = 0; end
    params = {
        'HdtMode',            'String', mode
        'PacketFormat',       'String', pktFmt
        'ModulationOrder',    'Int',    hdtParams.M
        'CodeRate',           'Double', hdtParams.codeRate
        'SymbolRate',         'Double', symbolRate
        'SamplesPerSymbol',   'Int',    sps
        'PayloadBlockLength', 'Int',    blockLen
        'ChannelIndex',       'Int',    channelIndex
        'PayloadLength',      'Int',    octets
        'PayloadHex',         'String', wg_bits2hex(data)
        };
    wg_write(outDir, stem, [pad; wave(:); pad], fs, fc, 'BluetoothLeHdt', 'helperBLEHDTWaveform', params);
end
end
