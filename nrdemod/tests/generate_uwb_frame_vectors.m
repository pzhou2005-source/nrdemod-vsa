function generate_uwb_frame_vectors(outputDir)
if nargin < 1, outputDir = fullfile('nrdemod', 'build', 'frame-tests', 'vectors'); end
if ~isfolder(outputDir), mkdir(outputDir); end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'tools', 'wavegen_common'));
wg_require_example('helperEHPRFWaveformGenerator', 'comm/IEEE802154abWaveformGenerationHRPUWBExample');
codes = cell(1, 32);
for index = 1:32, codes{index} = lrwpan.internal.HRPCodes(index); end
writeJson(fullfile(outputDir, 'codebook.json'), codes);
cases = {};
for index = [1:6 9:16 21:24]
    prf = 15.6;
    if index > 8, prf = 62.4; end
    cases{end+1} = lrwpanHRPConfig(Mode='802.15.4a', CodeIndex=index, MeanPRF=prf, PSDULength=13);
end
for index = 25:32
    cases{end+1} = lrwpanHRPConfig(Mode='HPRF', CodeIndex=index, PSDULength=13);
end
for prf = [3.9 15.6 62.4]
    rates = [0.11 0.85 6.81 27.24];
    if prf == 3.9, rates = [0.11 0.85 1.7 6.81]; end
    for rate = rates
        index = 3;
        if prf == 62.4, index = 9; end
        cfg = lrwpanHRPConfig(Mode='802.15.4a', CodeIndex=index, MeanPRF=prf, DataRate=rate, PSDULength=13);
        if prf == 3.9, cfg.PreambleMeanPRF = 4.03; end
        cases{end+1} = cfg;
    end
end
for sfd = 0:4
    cases{end+1} = lrwpanHRPConfig(Mode='HPRF', SFDNumber=sfd, ConstraintLength=7, PSDULength=13);
end
for units = [16 32 64 128 256]
    cases{end+1} = lrwpanHRPConfig(Mode='HPRF', STSPacketConfiguration=2, ...
        STSSegmentLength=units, NumSTSSegments=3, ExtraSTSGapLength=37, ExtraSTSGapIndex=2, PSDULength=13);
end
for rate = [1.95 7.8 31.2 62.4 124.8]
    for coding = {'BCC', 'LDPC'}
        cases{end+1} = helperEHPRFConfig(BitRate=rate, DynamicPHR=true, Coding=coding{1}, PSDULength=13);
    end
    cases{end+1} = helperEHPRFConfig(BitRate=rate, DynamicPHR=false, PSDULength=13);
end
for rate = [6.8 27.2]
    cases{end+1} = helperEHPRFConfig(BitRate=rate, PSDULength=13);
end
for count = [0 21 22 44 45 121 122 162 163 1024]
    cases{end+1} = helperEHPRFConfig(BitRate=124.8, DynamicPHR=true, Coding='LDPC', PSDULength=count);
end
for fcsLength = [2 4]
    cases{end+1} = lrwpanHRPConfig(Mode='HPRF', PSDULength=9+fcsLength);
end
for index = 1:numel(cases)
    cfg = cases{index};
    if any(cfg.CodeIndex == [3 4]), cfg.Channel = 9; end
    if any(cfg.CodeIndex == [5 6]), cfg.Channel = 3; end
    bytes = uint8(mod((0:cfg.PSDULength-1)*37+18, 256));
    fcsLength = 0;
    if index == numel(cases)-1
        bytes = uint8(['123456789' 137 33]);
        fcsLength = 2;
    elseif index == numel(cases)
        bytes = uint8(['123456789' 38 57 244 203]);
        fcsLength = 4;
    end
    bits = reshape(double(bitget(repmat(bytes(:), 1, 8), repmat(1:8, numel(bytes), 1)))', [], 1);
    options = struct(codeIndex=cfg.CodeIndex, sfdIndex=cfg.SFDNumber, ...
        legacy=strcmp(cfg.Mode, '802.15.4a'), samplesPerPulse=cfg.SamplesPerPulse, ...
        stsPacketConfig=cfg.STSPacketConfiguration, constraintLength=3, ...
        numStsSegments=cfg.NumSTSSegments, stsSegmentLength=cfg.STSSegmentLength, ...
        extraStsGapLength=cfg.ExtraSTSGapLength, extraStsGapIndex=cfg.ExtraSTSGapIndex, ...
        legacyVsaHopping=false, macFcsLength=fcsLength);
    mode = 0;
    if strcmp(cfg.Mode, 'EHPRF')
        mode = 2;
        options.dynamicPhr = cfg.DynamicPHR;
        options.bitRateMbps = cfg.BitRate;
        wave = helperEHPRFWaveformGenerator(bits, cfg);
    else
        options.highRatePhr = cfg.PHRDataRate > 1;
        if strcmp(cfg.Mode, 'HPRF')
            mode = 1;
            options.meanPrfMHz = cfg.MeanPRF;
            options.constraintLength = cfg.ConstraintLength;
        else
            options.legacyRateMbps = cfg.DataRate;
            options.legacyMeanPrfMHz = cfg.MeanPRF;
            options.sfdIndex = 0;
            options.stsPacketConfig = 0;
            options.numStsSegments = 1;
            options.stsSegmentLength = 64;
            options.extraStsGapLength = 0;
            options.extraStsGapIndex = 0;
        end
        wave = lrwpanWaveformGenerator(bits, cfg);
    end
    active = info(cfg);
    options.preambleSpread = active.PreambleSpreadingFactor;
    wave = [zeros(2048, 1); wave; zeros(2048, 1)];
    stem = sprintf('vector_%03d', index);
    fid = fopen(fullfile(outputDir, [stem '.bin']), 'wb');
    assert(fid >= 0);
    fwrite(fid, [real(wave) imag(wave)]', 'single');
    fclose(fid);
    metadata = struct(mode=mode, sampleRate=cfg.SampleRate, syncLength=cfg.PreambleDuration, ...
        frame=options, bytes=double(bytes), label=sprintf('%s code%d case%d', cfg.Mode, cfg.CodeIndex, index));
    writeJson(fullfile(outputDir, [stem '.json']), metadata);
end
fprintf('Generated %d independent frame vectors and 32 preamble tables\n', numel(cases));
end

function writeJson(path, value)
fid = fopen(path, 'w');
assert(fid >= 0);
fwrite(fid, jsonencode(value), 'char');
fclose(fid);
end