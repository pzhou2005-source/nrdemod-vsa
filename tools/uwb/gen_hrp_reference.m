function gen_hrp_reference(outRoot)
%GEN_HRP_REFERENCE IEEE 802.15.4a/4z (BPRF) HRP UWB reference vectors for the Python demodulator.
%   Generates waveforms with the Communications Toolbox (lrwpanWaveformGenerator) and
%   exports every intermediate stage so tools/uwb/verify_hrp_demod.py can check the
%   Python demodulator (nrdemod/python/examples/uwb_hrp.py) bit-exactly, stage by stage.
%   Output: <outRoot>/uwb_hrp/<name>.mat   (MAT v7, readable by scipy.io.loadmat)
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'wavegen_common'));
if nargin < 1, outRoot = wg_root(); end
outDir = fullfile(outRoot, 'uwb_hrp');
if ~exist(outDir, 'dir'), mkdir(outDir); end

cases = {
    'HRP_4z_BPRF_ch9_CI9_PHR850k_SP0_PSDU20',    {'Mode', 'BPRF', 'Channel', 9, 'CodeIndex', 9,  'PHRDataRate', 0.85, 'STSPacketConfiguration', 0, 'PSDULength', 20}
    'HRP_4z_BPRF_ch5_CI10_PHR6M8_SP0_PSDU127',   {'Mode', 'BPRF', 'Channel', 5, 'CodeIndex', 10, 'PHRDataRate', 6.81, 'STSPacketConfiguration', 0, 'PSDULength', 127, 'Ranging', true}
    'HRP_4z_BPRF_ch9_CI11_SFD2_SP1_PSDU50',      {'Mode', 'BPRF', 'Channel', 9, 'CodeIndex', 11, 'SFDNumber', 2, 'STSPacketConfiguration', 1, 'PSDULength', 50}
    'HRP_4z_BPRF_ch9_CI12_SFD0_SP2_PSDU33',      {'Mode', 'BPRF', 'Channel', 9, 'CodeIndex', 12, 'SFDNumber', 0, 'STSPacketConfiguration', 2, 'PSDULength', 33}
    'HRP_4a_ch9_CI3_PRF15M6_6M81_PSDU100',       {'Mode', '802.15.4a', 'Channel', 9, 'MeanPRF', 15.6, 'DataRate', 6.81, 'CodeIndex', 3, 'PSDULength', 100}
    'HRP_4a_ch9_CI3_PRF15M6_850k_PSDU30',        {'Mode', '802.15.4a', 'Channel', 9, 'MeanPRF', 15.6, 'DataRate', 0.85, 'CodeIndex', 3, 'PSDULength', 30}
    'HRP_4a_ch9_CI4_PRF15M6_110k_PSDU12',        {'Mode', '802.15.4a', 'Channel', 9, 'MeanPRF', 15.6, 'DataRate', 0.11, 'CodeIndex', 4, 'PSDULength', 12}
    'HRP_4a_ch9_CI9_PRF62M4_27M24_PSDU60',       {'Mode', '802.15.4a', 'Channel', 9, 'MeanPRF', 62.4, 'DataRate', 27.24, 'CodeIndex', 9, 'PSDULength', 60}
    'HRP_4a_ch9_CI3_PRF3M9_1M7_PSDU16',          {'Mode', '802.15.4a', 'Channel', 9, 'MeanPRF', 3.9, 'DataRate', 1.7, 'CodeIndex', 3, 'PSDULength', 16}
    };

rng(20260420);
fprintf('HRP reference -> %s\n', outDir);
for k = 1:size(cases, 1)
    name = cases{k, 1};
    cfg = lrwpanHRPConfig(cases{k, 2}{:});
    psdu = randi([0 1], 8 * cfg.PSDULength, 1);

    wave = lrwpanWaveformGenerator(psdu, cfg);
    [waveInt, chips] = lrwpan.internal.lrwpanHRPWaveformGenerator(psdu, cfg);
    assert(isequal(wave, waveInt), 'public/internal generator mismatch');

    % Intermediate stages (Sec. 15.3 of IEEE 802.15.4-2020)
    rsPsdu = lrwpan.internal.hrpRS(psdu, true);
    phrBits = phrWithSecded(numel(psdu), cfg);
    convIn = [phrBits; rsPsdu; zeros(2, 1)];
    convCw = convenc(convIn, poly2trellis(3, [2 5]));
    numSym = numel(convCw) / 2;

    code = lrwpan.internal.HRPCodes(cfg.CodeIndex);
    seed = code(code ~= 0);
    seed(seed == -1) = 0;
    seed = seed(1:15);
    cpb = cfg.ChipsPerBurst;
    numPn = 21 * cpb(1) + (numSym - 21) * cpb(end) + 64;
    pn = comm.PNSequence(Polynomial = '1 + D + D15', InitialConditions = fliplr(seed(:).'), ...
        SamplesPerFrame = numPn, Mask = 15);
    pnSeq = pn();

    ind = lrwpanHRPFieldIndices(cfg);
    s = struct();
    s.name = name;
    s.mode = char(cfg.Mode);
    s.channel = cfg.Channel;
    s.code_index = cfg.CodeIndex;
    s.preamble_code = code(:).';
    s.preamble_spreading_factor = cfg.PreambleSpreadingFactor;
    s.preamble_duration = cfg.PreambleDuration;
    s.sfd = lrwpan.internal.getSFD(cfg);
    s.sfd_number = cfg.SFDNumber;
    s.mean_prf = cfg.MeanPRF;
    s.data_rate_kbps = cfg.DataRateNum;
    s.chips_per_burst = cpb(:).';
    s.bursts_per_symbol = cfg.BurstsPerSymbol;
    s.num_hop_bursts = cfg.NumHopBursts;
    s.sts_packet_configuration = stsCfg(cfg);
    s.ranging = double(cfg.Ranging);
    s.samples_per_pulse = cfg.SamplesPerPulse;
    s.sample_rate = cfg.SampleRate;
    s.chip_rate = cfg.SampleRate / cfg.SamplesPerPulse;
    s.psdu_bits = psdu(:).';
    s.phr_bits = phrBits(:).';
    s.rs_psdu_bits = rsPsdu(:).';
    s.conv_codeword = convCw(:).';
    s.pn_sequence = pnSeq(:).';
    s.chips = chips(:).';
    s.waveform = wave(:).';
    s.field_names = fieldnames(ind).';
    for f = fieldnames(ind).'
        s.(['idx_' f{1}]) = double(ind.(f{1}));
    end
    save(fullfile(outDir, [name '.mat']), '-struct', 's', '-v7');
    fprintf('  %-44s %9d samples, %4d PSDU bits, %4d symbols\n', name, numel(wave), numel(psdu), numSym);
end
end

function c = stsCfg(cfg)
if strcmp(cfg.Mode, '802.15.4a'), c = 0; else, c = cfg.STSPacketConfiguration; end
end

function phr = phrWithSecded(psduLen, cfg)
% Sec. 15.2.7 (non-HPRF PHR) followed by SECDED.
phr = zeros(13, 1);
switch cfg.DataRateNum
    case 110,   phr(1:2) = [0; 0];
    case 850,   phr(1:2) = [0; 1];
    case 1700,  phr(1:2) = [1; 0];
    case 6810
        if cfg.MeanPRFNum == 3.9, phr(1:2) = [1; 1]; else, phr(1:2) = [1; 0]; end
    case 27240, phr(1:2) = [1; 1];
end
phr(3:9) = int2bit(psduLen / 8, 7);
phr(10) = double(cfg.Ranging);
switch cfg.PreambleDuration
    case 16,   phr(12:13) = [0; 0];
    case 64,   phr(12:13) = [0; 1];
    case 1024, phr(12:13) = [1; 0];
    case 4096, phr(12:13) = [1; 1];
end
phr = lrwpan.internal.hrpSECDED(phr);
end
