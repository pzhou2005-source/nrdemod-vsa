function gen_bt_bredr(outRoot)
%GEN_BT_BREDR Bluetooth Classic BR (GFSK) / EDR 2M (pi/4-DQPSK) / EDR 3M (8DPSK) ACL packets with
%   bluetoothWaveformGenerator (Bluetooth Toolbox).
%   Output: <outRoot>/bt_bredr/<name>.txt + .para2 (INPUT names follow the 89600 Bluetooth para2).
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'wavegen_common'));
if nargin < 1, outRoot = wg_root(); end
outDir = fullfile(outRoot, 'bt_bredr');

%        mode     packetType
cases = {
    'BR',    'DH1'
    'BR',    'DH3'
    'BR',    'DH5'
    'EDR2M', 'DH1'
    'EDR2M', 'DH5'
    'EDR3M', 'DH1'
    'EDR3M', 'DH5'
    };
sps = 8;
symbolRate = 1e6;
fc = 2402e6;

rng(2026);
fprintf('Bluetooth BR/EDR -> %s\n', outDir);
for k = 1:size(cases, 1)
    [mode, pktType] = cases{k, :};
    cfg = bluetoothWaveformConfig('Mode', mode, 'PacketType', pktType, 'SamplesPerSymbol', sps);
    octets = getPayloadLength(cfg);
    data = randi([0 1], 8 * octets, 1);
    wave = bluetoothWaveformGenerator(data, cfg);
    fs = symbolRate * sps;
    pad = zeros(round(10e-6 * fs), 1);

    switch mode
        case 'BR',    standard = 'BluetoothModChar'; modFormat = 'GFSK';           vsaType = pktType;
        case 'EDR2M', standard = 'Bluetooth';        modFormat = 'PI_OVER4_DQPSK'; vsaType = ['2' pktType];
        case 'EDR3M', standard = 'Bluetooth';        modFormat = 'D8PSK';          vsaType = ['3' pktType];
    end
    addr = char(cfg.DeviceAddress);
    stem = sprintf('BT_%s_%s_%dB', mode, pktType, octets);
    params = {
        'ModFormat',        'String', modFormat
        'PacketType',       'String', vsaType
        'ModulationIndex',  'Double', cfg.ModulationIndex
        'SymbolRate',       'Double', symbolRate
        'SamplesPerSymbol', 'Int',    sps
        'DeviceAddress',    'String', addr
        'LAP',              'String', addr(end-5:end)
        'PayloadLength',    'Int',    octets
        'PayloadHex',       'String', wg_bits2hex(data)
        };
    wg_write(outDir, stem, [pad; wave(:); pad], fs, fc, standard, 'bluetoothWaveformGenerator', params);
end
end
