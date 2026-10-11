function gen_ble(outRoot)
%GEN_BLE Bluetooth LE 1M / 2M / Coded S2 / Coded S8 packets with bleWaveformGenerator (Bluetooth Toolbox).
%   PDU = 16-bit data header + payload + CRC-24 (init 0x555555), default access address 0x8E89BED6.
%   Output: <outRoot>/ble/<name>.txt + .para2 (INPUT names follow the 89600 BluetoothLe para2).
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'wavegen_common'));
if nargin < 1, outRoot = wg_root(); end
outDir = fullfile(outRoot, 'ble');

%        mode      vsaPacketFormat symbolRate  octets  pattern
cases = {
    'LE1M',   1, 1e6,  37, 'PRBS9'
    'LE1M',   1, 1e6, 255, 'PRBS9'
    'LE1M',   1, 1e6,  37, '11110000'
    'LE1M',   1, 1e6,  37, '10101010'
    'LE2M',   2, 2e6,  37, 'PRBS9'
    'LE2M',   2, 2e6, 255, 'PRBS9'
    'LE500K', 3, 1e6,  37, 'PRBS9'
    'LE125K', 4, 1e6,  37, 'PRBS9'
    };
sps = 8;
channelIndex = 37;
crcInit = '555555';

fprintf('Bluetooth LE -> %s\n', outDir);
for k = 1:size(cases, 1)
    [mode, pktFmt, symbolRate, octets, pattern] = cases{k, :};
    payload = payload_bits(pattern, octets);
    header = [bitget(2, 1:8) bitget(octets, 1:8)].';  % LLID=2, NESN/SN/MD/CP=0, length
    pdu = [header; payload];
    pdu = [pdu; ble_crc24(pdu, hex2dec(crcInit))];

    wave = bleWaveformGenerator(pdu, 'Mode', mode, 'ChannelIndex', channelIndex, 'SamplesPerSymbol', sps);
    fs = symbolRate * sps;
    pad = zeros(round(10e-6 * fs), 1);

    stem = sprintf('BLE_%s_%s_%dB', mode, pattern, octets);
    params = {
        'PacketFormat',     'Int',    pktFmt
        'SymbolRate',       'Double', symbolRate
        'ModulationIndex',  'Double', 0.5
        'Mode',             'String', mode
        'SamplesPerSymbol', 'Int',    sps
        'ChannelIndex',     'Int',    channelIndex
        'AccessAddress',    'String', '8E89BED6'
        'CrcInit',          'String', crcInit
        'PayloadLength',    'Int',    octets
        'PayloadPattern',   'String', pattern
        'PduHex',           'String', wg_bits2hex(pdu)
        };
    wg_write(outDir, stem, [pad; wave(:); pad], fs, wg_ble_rf_hz(channelIndex), 'BluetoothLe', 'bleWaveformGenerator', params);
end
end

function bits = payload_bits(pattern, octets)
n = 8 * octets;
switch pattern
    case 'PRBS9'
        reg = ones(1, 9);
        bits = zeros(n, 1);
        for i = 1:n  % x^9 + x^5 + 1, all-ones seed (Core spec Vol 6 Part F)
            bits(i) = reg(1);
            fb = xor(reg(1), reg(5));
            reg = [reg(2:end) fb];
        end
    case '11110000'
        bits = repmat([1 1 1 1 0 0 0 0].', octets, 1);
    case '10101010'
        bits = repmat([1 0 1 0 1 0 1 0].', octets, 1);
    otherwise
        error('gen_ble:pattern', 'unknown payload pattern %s', pattern);
end
end

function crc = ble_crc24(bits, init)
% Core spec Vol 6 Part B 3.1.1: x^24+x^10+x^9+x^6+x^4+x^3+x+1, position 0 = LSB of init,
% transmitted from position 23 down to position 0.
reg = bitget(init, 1:24);
for b = bits(:).'
    fb = xor(b, reg(24));
    reg = [fb reg(1:23)];
    taps = [1 3 4 6 9 10] + 1;
    reg(taps) = xor(reg(taps), fb);
end
crc = double(fliplr(reg)).';
end
