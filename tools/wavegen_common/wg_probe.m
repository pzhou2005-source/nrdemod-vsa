function wg_probe()
%WG_PROBE List which waveform-generation entry points this MATLAB install provides.
probes = {
    % standard,                      function,                    product
    '5G NR',                         'nrWaveformGenerator',       '5G Toolbox'
    'LTE / LTE-M',                   'lteRMCDLTool',              'LTE Toolbox'
    'NB-IoT',                        'lteNPDSCH',                 'LTE Toolbox'
    'Wi-Fi 4/5/6/7, 11a/b/g/ad',     'wlanWaveformGenerator',     'WLAN Toolbox'
    'Bluetooth LE',                  'bleWaveformGenerator',      'Bluetooth Toolbox'
    'Bluetooth BR/EDR',              'bluetoothWaveformGenerator','Bluetooth Toolbox'
    'UWB 802.15.4a/z (HRP)',         'lrwpanWaveformGenerator',   'Communications Toolbox'
    'ZigBee 802.15.4 O-QPSK',        'lrwpanOQPSKConfig',         'Communications Toolbox'
    'Generic OFDM',                  'comm.OFDMModulator',        'Communications Toolbox'
    'GFSK/CPFSK (SparkLink, BLE-like)', 'comm.CPFSKModulator',    'Communications Toolbox'
    'GMSK (GSM-like)',               'comm.GMSKModulator',        'Communications Toolbox'
    'APSK/QAM/PSK + RRC',            'comm.RaisedCosineTransmitFilter', 'Communications Toolbox'
    'DVB-S2/S2X',                    'dvbs2WaveformGenerator',    'Satellite Communications Toolbox'
    'GNSS (GPS/Galileo)',            'gpsWaveformGenerator',      'Satellite Communications Toolbox'
    'Wireless Waveform Generator app', 'wirelessWaveformGenerator', 'Communications Toolbox'
    };
fprintf('%-34s %-34s %-32s %s\n', 'Standard', 'Entry point', 'Product', 'Available');
for k = 1:size(probes, 1)
    ok = ~isempty(which(probes{k, 2}));
    fprintf('%-34s %-34s %-32s %s\n', probes{k, :}, string(ok));
end
end
