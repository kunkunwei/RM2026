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

# search vx and wz in a grid to find payload where exists x with calc_crc8(payload + x) == x
found_any = False
for vx_int in range(-20,21):
    vx = float(vx_int) / 10.0  # -2.0 .. 2.0 step 0.1
    for wz_int in range(-20,21):
        wz = float(wz_int) / 10.0
        payload = struct.pack('<f', vx) + struct.pack('<f', wz)
        candidates = [x for x in range(256) if calc_crc8(payload + bytes([x])) == x]
        if candidates:
            print('Found:', vx, wz)
            print('payload hex:', ' '.join(f'{b:02X}' for b in payload))
            print('crc candidates:', ' '.join(f'{c:02X}' for c in candidates))
            x = candidates[0]
            packet = payload + bytes([x])
            print('full packet hex:', ' '.join(f'{b:02X}' for b in packet))
            print('Device ASCII response (usbDebug_float on vx):', '{:.2f}\n'.format(vx))
            found_any = True
            raise SystemExit(0)

if not found_any:
    print('No valid packet found in the scanned range')

