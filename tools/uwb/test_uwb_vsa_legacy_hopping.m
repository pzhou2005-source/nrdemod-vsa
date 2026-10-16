function tests = test_uwb_vsa_legacy_hopping
tests = functiontests(localfunctions);
end

function testShortBurst(testCase)
cfg = legacyConfig(6.81);
psdu = zeros(8 * cfg.PSDULength, 1);
[originalWave, originalSymbols] = lrwpan.internal.lrwpanHRPWaveformGenerator(psdu, cfg);
[wave, adjusted, symbols] = uwb_vsa_legacy_hopping(psdu, cfg);
verifyTrue(testCase, adjusted);
verifySize(testCase, wave, size(originalWave));
active = info(cfg);
headerChips = (cfg.PreambleDuration + numel(lrwpan.internal.getSFD(cfg))) ...
    * active.PreambleCodeLength * active.PreambleSpreadingFactor ...
    + (19 + cfg.ConstraintLength - 1) * active.ChipsPerSymbol(1);
verifyEqual(testCase, symbols(1:headerChips), originalSymbols(1:headerChips));
verifyEqual(testCase, wave(1:headerChips * cfg.SamplesPerPulse), ...
    originalWave(1:headerChips * cfg.SamplesPerPulse));
originalPayload = reshape(originalSymbols(headerChips + 1:end), 64, []);
payload = reshape(symbols(headerChips + 1:end), 64, []);
hops = zeros(1, size(payload, 2));
for symbolIndex = 1:size(payload, 2)
    positions = find(payload(:, symbolIndex));
    originalPositions = find(originalPayload(:, symbolIndex));
    verifyEqual(testCase, numel(positions), 2);
    verifyEqual(testCase, payload(positions, symbolIndex), ...
        originalPayload(originalPositions, symbolIndex));
    verifyEqual(testCase, floor((positions(1) - 1) / 32), ...
        floor((originalPositions(1) - 1) / 32));
    hops(symbolIndex) = mod(positions(1) - 1, 32) / 2;
end
verifyEqual(testCase, hops(1:16), [7 7 5 3 6 1 2 0 0 6 5 3 2 6 1 0]);
end

function testOtherLegacyRatesUnchanged(testCase)
for rate = [0.85 27.24]
    cfg = legacyConfig(rate);
    psdu = zeros(8 * cfg.PSDULength, 1);
    [wave, adjusted] = uwb_vsa_legacy_hopping(psdu, cfg);
    verifyFalse(testCase, adjusted);
    verifyEqual(testCase, wave, complex(lrwpanWaveformGenerator(psdu, cfg)));
end
end

function testBprfUnchanged(testCase)
cfg = lrwpanHRPConfig('Mode', 'BPRF', 'PSDULength', 100);
psdu = zeros(8 * cfg.PSDULength, 1);
[wave, adjusted] = uwb_vsa_legacy_hopping(psdu, cfg);
verifyFalse(testCase, adjusted);
verifyEqual(testCase, wave, complex(lrwpanWaveformGenerator(psdu, cfg)));
end

function cfg = legacyConfig(rate)
cfg = lrwpanHRPConfig('Mode', '802.15.4a', 'Channel', 9, ...
    'MeanPRF', 15.6, 'DataRate', rate, 'CodeIndex', 3, 'PSDULength', 100);
end