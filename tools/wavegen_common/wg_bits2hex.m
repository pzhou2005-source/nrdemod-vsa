function h = wg_bits2hex(bits)
%WG_BITS2HEX Octets of a bit vector as hex, first bit of each octet = LSB (air order).
bits = double(bits(:));
bits = [bits; zeros(mod(-numel(bits), 8), 1)];
bytes = (2 .^ (0:7)) * reshape(bits, 8, []);
h = reshape(dec2hex(bytes, 2).', 1, []);
end
