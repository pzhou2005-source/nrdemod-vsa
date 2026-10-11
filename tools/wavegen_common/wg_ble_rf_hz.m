function f = wg_ble_rf_hz(channelIndex)
%WG_BLE_RF_HZ Bluetooth LE channel index (0..39) -> RF centre frequency in Hz.
switch channelIndex
    case 37, f = 2402e6;
    case 38, f = 2426e6;
    case 39, f = 2480e6;
    otherwise
        if channelIndex <= 10
            f = 2404e6 + 2e6 * channelIndex;
        else
            f = 2428e6 + 2e6 * (channelIndex - 11);
        end
end
end
