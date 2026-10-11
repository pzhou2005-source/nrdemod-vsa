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

rng(2026);
fprintf('5G NR -> %s\n', outDir);
for k = 1:size(cases, 1)
    [link, fr, bw, scs, nrb, nprb, modName, tp, fc] = cases{k, :};
    mu = log2(scs / 15);
    nSlots = numSubframes * 2^mu;

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

    tpTag = '';
    if tp, tpTag = '_DFTs'; end
    stem = sprintf('5GNR_%s_%s_%dMHz_mu%d_%dPRB_%s%s', fr, link, bw, mu, nprb, modName, tpTag);
    params = {
        'link',                          'String', link
        'numerology',                    'Int',    mu
        'frequencyRange',                'Int',    sscanf(fr, 'FR%d')
        'channelBandwidthMHz',           'Int',    bw
        'subcarrierSpacingKHz',          'Int',    scs
        'fftSize',                       'Int',    gi.Nfft
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
        'targetCodeRate',                'Double', ch.TargetCodeRate
        'nCellID',                       'Int',    cfg.NCellID
        };
    wg_write(outDir, stem, wave(:, 1), fs, fc, ['NR5G' link], 'nrWaveformGenerator', params);
end
end
