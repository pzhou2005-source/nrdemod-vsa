function gen_hprf_reference(outRoot)
%GEN_HPRF_REFERENCE Independent CL3/CL7 vectors for the native HRP decoder.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'wavegen_common'));
if nargin < 1, outRoot = wg_root(); end
outDir = fullfile(outRoot, 'uwb_hprf');
if ~exist(outDir, 'dir'), mkdir(outDir); end
cases = {
    'HPRF249_CL3_SP0', {'MeanPRF',249.6,'ConstraintLength',3,'CodeIndex',25,'SFDNumber',2,'STSPacketConfiguration',0,'PSDULength',127}
    'HPRF124_CL3_SP1', {'MeanPRF',124.8,'ConstraintLength',3,'CodeIndex',26,'SFDNumber',4,'STSPacketConfiguration',1,'NumSTSSegments',4,'STSSegmentLength',16,'PSDULength',1024}
    'HPRF249_CL7_SP2', {'MeanPRF',249.6,'ConstraintLength',7,'CodeIndex',31,'SFDNumber',3,'PreambleDuration',32,'STSPacketConfiguration',2,'ExtraSTSGapIndex',1,'ExtraSTSGapLength',127,'PSDULength',1023}
    'HPRF124_CL7_SP0', {'MeanPRF',124.8,'ConstraintLength',7,'CodeIndex',32,'SFDNumber',1,'PreambleDuration',16,'STSPacketConfiguration',0,'PSDULength',1025}
    'HPRF249_CL7_MAX', {'MeanPRF',249.6,'ConstraintLength',7,'CodeIndex',28,'SFDNumber',4,'PreambleDuration',24,'STSPacketConfiguration',0,'PSDULength',4095}
    };
rng(20261015);
for k = 1:size(cases,1)
    cfg = lrwpanHRPConfig('Mode','HPRF','Channel',9,cases{k,2}{:});
    psdu = randi([0,1],8*cfg.PSDULength,1);
    % The 124.8 MHz mapper already inserts half-rate gaps; export on the chip grid.
    s = struct('name',cases{k,1},'waveform',lrwpanWaveformGenerator(psdu,cfg).', ...
        'psdu_bits',psdu.','sample_rate',cfg.SamplesPerPulse*499.2e6, ...
        'samples_per_pulse',cfg.SamplesPerPulse,'code_index',cfg.CodeIndex, ...
        'sfd_number',cfg.SFDNumber,'preamble_duration',cfg.PreambleDuration, ...
        'preamble_spreading_factor',cfg.PreambleSpreadingFactor,'mean_prf',cfg.MeanPRF, ...
        'constraint_length',cfg.ConstraintLength,'sts_packet_configuration',cfg.STSPacketConfiguration, ...
        'num_sts_segments',cfg.NumSTSSegments,'sts_segment_length',cfg.STSSegmentLength, ...
        'extra_sts_gap_index',cfg.ExtraSTSGapIndex,'extra_sts_gap_length',cfg.ExtraSTSGapLength);
    idx = lrwpanHRPFieldIndices(cfg);
    s.idx_PHR = idx.PHR;
    save(fullfile(outDir,[s.name '.mat']),'-struct','s','-v7');
    fprintf('%s: %d octets, %d samples on 499.2 MHz grid\n',s.name,cfg.PSDULength,numel(s.waveform));
end
end
