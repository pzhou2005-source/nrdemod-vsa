function gen_sparklink(outRoot)
%GEN_SPARKLINK SparkLink (NearLink) SLE GFSK bursts, toolbox-free.
%   No MathWorks generator exists for SLE, so this models the SLE GFSK physical layer only
%   (1/2/4 Msym/s, BT=0.5, h=0.5) on a simplified frame: alternating preamble, 32-bit sync
%   word, raw payload (no polar coding / whitening / CRC). Replace frame_bits() with an
%   OpenSparkLink bit stream when spec-conformant framing is needed.
%   Output: <outRoot>/sparklink/<name>.txt + .para2.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'wavegen_common'));
if nargin < 1, outRoot = wg_root(); end
outDir = fullfile(outRoot, 'sparklink');

%        symbolRate  octets
cases = {
    1e6,  37
    1e6, 255
    2e6,  37
    2e6, 255
    4e6,  37
    4e6, 255
    };
sps = 8;
h = 0.5;
bt = 0.5;
syncWord = '5A3C96E1';  % placeholder access code
fc = 2440e6;

rng(2026);
fprintf('SparkLink SLE -> %s\n', outDir);
for k = 1:size(cases, 1)
    [symbolRate, octets] = cases{k, :};
    preambleLen = 8 * symbolRate / 1e6;  % keeps the preamble at 8 us
    payload = randi([0 1], 8 * octets, 1);
    bits = [mod((0:preambleLen-1).', 2); hex2bits(syncWord); payload];
    wave = gfsk(bits, sps, h, bt);
    fs = symbolRate * sps;
    pad = zeros(round(10e-6 * fs), 1);

    stem = sprintf('SLE_GFSK_%dM_%dB', symbolRate / 1e6, octets);
    params = {
        'SymbolRate',       'Double', symbolRate
        'ModulationIndex',  'Double', h
        'BT',               'Double', bt
        'SamplesPerSymbol', 'Int',    sps
        'PreambleLength',   'Int',    preambleLen
        'SyncWord',         'String', syncWord
        'PayloadLength',    'Int',    octets
        'PayloadHex',       'String', wg_bits2hex(payload)
        'Framing',          'String', 'simplified-uncoded'
        };
    wg_write(outDir, stem, [pad; wave; pad], fs, fc, 'SparkLinkSle', 'gen_sparklink/gfsk', params);
end
end

function bits = hex2bits(hex)
% octets in string order, LSB of each octet first (air order)
bytes = hex2dec(reshape(hex, 2, []).');
bits = reshape(dec2bin(bytes, 8).' == '1', 8, []);
bits = double(reshape(flipud(bits), [], 1));
end

function s = gfsk(bits, sps, h, bt)
span = 4;  % Gaussian span in symbols
t = (-span * sps / 2:span * sps / 2).' / sps;
g = exp(-2 * pi^2 * bt^2 * t.^2 / log(2));
g = g / sum(g);
nrz = repelem(2 * bits(:) - 1, sps);
f = conv(nrz, g, 'same');
s = exp(1j * pi * h * cumsum(f) / sps);
end
