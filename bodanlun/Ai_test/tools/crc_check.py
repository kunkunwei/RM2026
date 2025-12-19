# Quick CRC8 and float decode check for the USB packet
# Re-implements the C calculate_crc8 from usb.c and finds crc byte x such that
# calculate_crc8(payload + x) == x.
import struct

def calc_crc8(data: bytes) -> int:
    crc = 0
    poly = 0x07
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ poly
            else:
                crc = (crc << 1) & 0xFF
    return crc

# payload bytes from user: 31 2E 34 20 35 2E 38 20 3F
buf = bytes([0x31,0x2E,0x34,0x20,0x35,0x2E,0x38,0x20])
print('first 8 bytes hex:', ' '.join(f'{b:02X}' for b in buf))

# find x in 0..255 such that calc_crc8(buf + bytes([x])) == x
found = []
for x in range(256):
    if calc_crc8(buf + bytes([x])) == x:
        found.append(x)

print('valid crc candidates count:', len(found))
if found:
    print('candidates (hex):', ' '.join(f'{x:02X}' for x in found))
    x = found[0]
else:
    x = None

# compute the float values that stm32 will memcpy from the first 8 bytes
vx_bytes = buf[0:4]
wz_bytes = buf[4:8]
# interpret as little-endian float
vx = struct.unpack('<f', vx_bytes)[0]
wz = struct.unpack('<f', wz_bytes)[0]
print('vx raw bytes:', ' '.join(f'{b:02X}' for b in vx_bytes), '-> float', vx)
print('wz raw bytes:', ' '.join(f'{b:02X}' for b in wz_bytes), '-> float', wz)

# what the device will send on success: usbDebug_float(vx_set) -> "%.2f\n"
if x is not None:
    to_send = buf + bytes([x])
    print('\nSuggested 9-byte packet to send (hex):', ' '.join(f'{b:02X}' for b in to_send))
else:
    print('\nNo self-consistent crc found for these 8 bytes (with current CRC code).')

# compute ascii response bytes as hex
resp = ('{:.2f}\n'.format(vx)).encode('ascii', errors='replace')
print('Device ASCII response if success (text):', resp)
print('Device ASCII response hex:', ' '.join(f'{b:02X}' for b in resp))

# Also print what usbDebug_uint(666) would look like
u = '666\r\n'.encode('ascii')
print('On CRC error the code sends usbDebug_uint(666) (hex):', ' '.join(f'{b:02X}' for b in u))

# Finally compute calculate_crc8 over the full original 9 bytes the user actually sent (including 0x3F)
original = buf + bytes([0x3F])
print('\nOriginal 9-byte CRC calc:', calc_crc8(original))
print('Original last byte (crc provided): 0x3F')
print('calc_crc8(original) == last?', calc_crc8(original) == 0x3F)

