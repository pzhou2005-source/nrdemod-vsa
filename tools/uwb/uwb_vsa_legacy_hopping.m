function [wave, adjusted, symbols] = uwb_vsa_legacy_hopping(psdu, cfg)
%UWB_VSA_LEGACY_HOPPING Match VSA's short-burst hopping window.
%   This is an explicit bench interoperability convention, not an independent
%   certification of IEEE 802.15.4 conformance. Spreading and FEC are unchanged.
[wave, symbols] = lrwpan.internal.lrwpanHRPWaveformGenerator(psdu, cfg);
adjusted = false;
if ~strcmp(cfg.Mode, '802.15.4a')
    return;
end
active = info(cfg);
burstChips = active.ChipsPerBurst(end);
hopBits = log2(active.NumHopBursts);
if burstChips ~= 2 || active.NumHopBursts ~= 8
    return;
end
headerSymbols = 19 + cfg.ConstraintLength - 1;
headerPnChips = headerSymbols * active.ChipsPerBurst(1);
shrChips = (cfg.PreambleDuration + numel(lrwpan.internal.getSFD(cfg))) ...
    * active.PreambleCodeLength * active.PreambleSpreadingFactor;
headerChips = shrChips + headerSymbols * active.ChipsPerSymbol(1);
symbolChips = active.ChipsPerSymbol(end);
payload = reshape(symbols(headerChips + 1:end), symbolChips, []);
code = lrwpan.internal.HRPCodes(cfg.CodeIndex);
seed = code(code ~= 0);
seed = fliplr(double(seed(1:15) > 0));
pn = comm.PNSequence('Polynomial', '1 + D + D15', ...
    'InitialConditions', seed, 'Mask', 15, ...
    'SamplesPerFrame', headerPnChips + size(payload, 2) * burstChips);
sequence = pn();
for symbolIndex = 1:size(payload, 2)
    positions = find(payload(:, symbolIndex));
    assert(numel(positions) == burstChips, 'Unexpected legacy burst length');
    half = floor((positions(1) - 1) / (symbolChips / 2));
    window = headerPnChips + (symbolIndex - 1) * burstChips + 1 ...
        - (hopBits - burstChips);
    hop = (2 .^ (0:hopBits - 1)) * sequence(window:window + hopBits - 1);
    first = 1 + burstChips * (hop + half * active.BurstsPerSymbol / 2);
    burst = payload(positions, symbolIndex);
    payload(:, symbolIndex) = 0;
    payload(first:first + burstChips - 1, symbolIndex) = burst;
end
symbols(headerChips + 1:end) = payload(:);
impulses = zeros(numel(symbols) * cfg.SamplesPerPulse, 1);
impulses(1:cfg.SamplesPerPulse:end) = symbols;
[numerator, denominator] = butter(4, 1 / cfg.SamplesPerPulse);
wave = complex(filter(numerator, denominator, impulses));
adjusted = true;
end