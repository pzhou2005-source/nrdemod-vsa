function wg_write(outDir, stem, iq, fs, fc, standard, generator, params)
%WG_WRITE Write <stem>.txt (89600 text recording) and <stem>.para2 into outDir.
%   params is an N-by-3 cell {name, type, value}; type is Bool, Int, Double or String.
%   The .txt layout matches captures/tdc_reference_plaintext so tools/legacy_correlate.py
%   (parse_waveform) and nrdemod.vsa.load_iq read it unchanged.

if ~exist(outDir, 'dir')
    mkdir(outDir);
end
iq = double(iq(:));
pk = max(abs([real(iq); imag(iq)]));
if pk > 0
    iq = iq * (0.5 / pk);
end

fid = fopen(fullfile(outDir, [stem '.txt']), 'w');
assert(fid > 0, 'cannot write %s', fullfile(outDir, [stem '.txt']));
fprintf(fid, 'InputZoom\tTRUE\nInputCenter\t%.17g\nInputRefImped\t50.0\n', fc);
fprintf(fid, 'XStart\t0.0\nXDelta\t%.17g\nXDomain\t2\nXUnit\tSec\nYUnit\tV\n', 1 / fs);
fprintf(fid, 'TimeString\t%s\nY\n', datestr(now, 'ddd mmm dd HH:MM:SS yyyy'));
fprintf(fid, '%.10f\t%.10f\n', [real(iq) imag(iq)].');
fclose(fid);

fid = fopen(fullfile(outDir, [stem '.para2']), 'w');
assert(fid > 0, 'cannot write %s', fullfile(outDir, [stem '.para2']));
fprintf(fid, 'STANDARD, %s, , \n', standard);
for k = 1:size(params, 1)
    fprintf(fid, 'INPUT, %s, %s, %s\n', params{k, 1}, params{k, 2}, fmt(params{k, 2}, params{k, 3}));
end
% INFO lines are ignored by legacy_correlate.parse_para2 (it only reads RESULT)
fprintf(fid, 'INFO, Generator, String, %s\n', generator);
fprintf(fid, 'INFO, MatlabVersion, String, %s\n', version);
fprintf(fid, 'INFO, SampleRate, Double, %s\n', fmt('Double', fs));
fprintf(fid, 'INFO, CenterFrequency, Double, %s\n', fmt('Double', fc));
fprintf(fid, 'INFO, NumSamples, Int, %d\n', numel(iq));
fprintf(fid, 'INFO, WaveformFile, String, %s.txt\n', stem);
fclose(fid);

fprintf('  %-48s %9d samples @ %.6g MS/s\n', stem, numel(iq), fs / 1e6);
end

function s = fmt(type, v)
switch type
    case 'Bool'
        s = sprintf('%d', logical(v));
    case 'Int'
        s = sprintf('%d', round(v));
    case 'Double'
        if v ~= 0 && abs(v) < 1e-3
            s = sprintf('%.9e', v);
        else
            s = sprintf('%.6f', v);
        end
    case 'String'
        if ischar(v)
            s = v;
        else
            s = num2str(v);
        end
    otherwise
        error('wg_write:type', 'unknown para2 type %s', type);
end
end
