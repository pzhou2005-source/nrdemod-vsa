function gen_nr5g(outRoot)
%GEN_NR5G 5G NR DL (PDSCH) / UL (PUSCH) test waveforms with nrWaveformGenerator (5G Toolbox).
%   Output: <outRoot>/nr5g/<name>.txt + .para2. The para2 INPUT names are the
%   nrdemod.Config fields, so the file doubles as the demodulator configuration.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'wavegen_common'));
if nargin < 1, outRoot = wg_root(); end
outDir = fullfile(outRoot, 'nr5g');

%        link  FR     BW  SCS  carrierRB allocRB modulation transformPrecoding  fc
cases = {
    'DL', 'FR1',  20,  15, 106, 106, '256QAM', false, 3.5e9
    'DL', 'FR1', 100,  30, 273, 273, '64QAM',  false, 3.5e9
    'DL', 'FR1', 100,  30, 273, 273, '256QAM', false, 3.5e9
    'DL', 'FR2', 200, 120, 132, 132, '16QAM',  false, 28e9
    'UL', 'FR1',  50,  30, 133, 133, '64QAM',  false, 3.5e9
    'UL', 'FR1',  50,  30, 133, 120, 'QPSK',   true,  3.5e9
    'UL', 'FR1', 100,  30, 273, 270, '16QAM',  true,  3.5e9
    'UL', 'FR2', 100, 120,  66,  64, '64QAM',  false, 28e9
    };
modCode = containers.Map({'QPSK', '16QAM', '64QAM', '256QAM'}, {1, 2, 3, 4});
numSubframes = 1;

nid = 1;  % NCellID = DM-RS scrambling identity
hasToolbox = ~isempty(which('nrWaveformGenerator'));
if hasToolbox
    generator = 'nrWaveformGenerator';
else
    generator = 'gen_nr5g/nr_ofdm (TS 38.211 CP-OFDM, uncoded; 5G Toolbox not installed)';
end

rng(2026);
fprintf('5G NR -> %s (%s)\n', outDir, generator);
for k = 1:size(cases, 1)
    [link, fr, bw, scs, nrb, nprb, modName, tp, fc] = cases{k, :};
    mu = log2(scs / 15);
    nSlots = numSubframes * 2^mu;

    if hasToolbox
        [wave, fs, nfft, codeRate] = nr_toolbox(link, fr, bw, scs, nrb, nprb, modName, tp, fc, numSubframes, nSlots, nid);
    else
        [wave, fs, nfft] = nr_ofdm(nrb, nprb, scs, modName, tp, nSlots, nid);
        codeRate = 1;
    end

    tpTag = '';
    if tp, tpTag = '_DFTs'; end
    stem = sprintf('5GNR_%s_%s_%dMHz_mu%d_%dPRB_%s%s', fr, link, bw, mu, nprb, modName, tpTag);
    params = {
        'link',                          'String', link
        'numerology',                    'Int',    mu
        'frequencyRange',                'Int',    sscanf(fr, 'FR%d')
        'channelBandwidthMHz',           'Int',    bw
        'subcarrierSpacingKHz',          'Int',    scs
        'fftSize',                       'Int',    nfft
        'carrierResourceBlocks',         'Int',    nrb
        'numResourceBlocks',             'Int',    nprb
        'resourceBlockOffset',           'Int',    0
        'modulation',                    'Int',    modCode(modName)
        'modulationName',                'String', modName
        'transformPrecoding',            'Int',    tp
        'maxSymbols',                    'Int',    14 * nSlots
        'carrierOffsetHz',               'Double', 0
        'sampleRate',                    'Double', fs
        'symbolTimingAdjustmentPercent', 'Double', -3.125
        'dmrsExclude',                   'Int',    1
        'dmrsMappingTypeB',              'Int',    0
        'dmrsTypeAPosition',             'Int',    2
        'dmrsAdditionalPositions',       'Int',    0
        'dmrsDoubleSymbol',              'Int',    0
        'targetCodeRate',                'Double', codeRate
        'nCellID',                       'Int',    nid
        };
    wg_write(outDir, stem, wave(:, 1), fs, fc, ['NR5G' link], generator, params);
end
end

function [wave, fs, nfft, codeRate] = nr_toolbox(link, fr, bw, scs, nrb, nprb, modName, tp, fc, numSubframes, nSlots, nid)
if strcmp(link, 'DL')
    cfg = nrDLCarrierConfig;
    cfg.SSBurst.Enable = false;
    cfg.PDCCH{1}.Enable = false;
    cfg.CSIRS{1}.Enable = false;
    ch = nrWavegenPDSCHConfig('Modulation', modName, 'PRBSet', 0:nprb-1, ...
        'SymbolAllocation', [0 14], 'SlotAllocation', 0:nSlots-1);
    ch.ReservedCORESET = [];
else
    cfg = nrULCarrierConfig;
    cfg.PUCCH{1}.Enable = false;
    cfg.SRS{1}.Enable = false;
    ch = nrWavegenPUSCHConfig('Modulation', modName, 'PRBSet', 0:nprb-1, ...
        'SymbolAllocation', [0 14], 'SlotAllocation', 0:nSlots-1, 'TransformPrecoding', tp);
end
ch.DMRS.DMRSTypeAPosition = 2;
ch.DMRS.DMRSAdditionalPosition = 0;
ch.DMRS.DMRSLength = 1;

cfg.NCellID = nid;
cfg.FrequencyRange = fr;
cfg.ChannelBandwidth = bw;
cfg.NumSubframes = numSubframes;
cfg.CarrierFrequency = fc;
cfg.SCSCarriers = {nrSCSCarrierConfig('SubcarrierSpacing', scs, 'NStartGrid', 0, 'NSizeGrid', nrb)};
cfg.BandwidthParts = {nrWavegenBWPConfig('SubcarrierSpacing', scs, 'NStartBWP', 0, 'NSizeBWP', nrb)};
if strcmp(link, 'DL')
    cfg.PDSCH = {ch};
else
    cfg.PUSCH = {ch};
end

[wave, info] = nrWaveformGenerator(cfg);
gi = info.ResourceGrids(1).Info;
fs = gi.SampleRate;
nfft = gi.Nfft;
codeRate = ch.TargetCodeRate;
end

function [wave, fs, nfft] = nr_ofdm(nrb, nprb, scs, modName, tp, nSlots, nid)
% One single-layer PDSCH/PUSCH per slot over all 14 symbols, DM-RS type 1 on symbol 2
% (CDM group 0, no data in the DM-RS symbol), random QAM data, no channel coding.
mu = log2(scs / 15);
nfft = max(128, 2^ceil(log2(12 * nrb / 0.85)));
fs = nfft * scs * 1e3;
numSc = 12 * nprb;
bins = mod(-6 * nrb + (0:numSc - 1), nfft) + 1;
cpN = 144 * nfft / 2048;
cpL = cpN + 16 * nfft / 2048 * 2^mu;
M = sscanf(modName, '%dQAM');
if isempty(M), M = 4; end
L = sqrt(M);

wave = zeros(0, 1);
for slot = 0:nSlots - 1
    for l = 0:13
        if l == 2
            if tp
                r = low_papr(numSc / 2, nid);
            else
                cinit = mod(2^17 * (14 * slot + l + 1) * (2 * nid + 1) + 2 * nid, 2^31);
                c = gold(cinit, numSc);
                r = ((1 - 2 * c(1:2:end)) + 1j * (1 - 2 * c(2:2:end))).' / sqrt(2);
            end
            d = zeros(numSc, 1);
            d(1:2:end) = r;
        else
            lv = 2 * randi([0 L - 1], numSc, 2) - (L - 1);
            d = (lv(:, 1) + 1j * lv(:, 2)) / sqrt(2 * (M - 1) / 3);
            if tp
                d = fft(d) / sqrt(numSc);
            end
        end
        X = zeros(nfft, 1);
        X(bins) = d;
        t = ifft(X) * sqrt(nfft);
        cp = cpN;
        if mod(14 * slot + l, 7 * 2^mu) == 0, cp = cpL; end
        wave = [wave; t(end - cp + 1:end); t]; %#ok<AGROW>
    end
end
end

function c = gold(cinit, len)
% TS 38.211 5.2.1
Nc = 1600;
x1 = zeros(1, Nc + len + 31);
x2 = x1;
x1(1) = 1;
x2(1:31) = bitget(cinit, 1:31);
for n = 1:Nc + len
    x1(n + 31) = mod(x1(n + 3) + x1(n), 2);
    x2(n + 31) = mod(x2(n + 3) + x2(n + 2) + x2(n + 1) + x2(n), 2);
end
c = mod(x1(Nc + 1:Nc + len) + x2(Nc + 1:Nc + len), 2);
end

function r = low_papr(Mzc, nid)
% TS 38.211 5.2.2 type 1 low-PAPR sequence, no group/sequence hopping (u = nid mod 30, v = 0)
p = primes(Mzc - 1);
Nzc = p(end);
qbar = Nzc * (mod(nid, 30) + 1) / 31;
q = floor(qbar + 0.5);
m = 0:Nzc - 1;
x = exp(-1j * pi * q * m .* (m + 1) / Nzc);
r = x(mod(0:Mzc - 1, Nzc) + 1).';
end
