import pathlib, struct, zlib
root = pathlib.Path(__file__).resolve().parents[1]
packets = [bytes.fromhex(line) for line in (root/'reports/network_packets.hex').read_text().splitlines()]
assert len(packets) == 12
seen = set()
for p in packets:
    assert p[:8] == bytes.fromhex('55555555555555d5')
    eth = p[8:-4]
    assert zlib.crc32(eth).to_bytes(4,'little') == p[-4:], 'Ethernet FCS'
    ip = eth[14:34]
    checksum = sum(struct.unpack('!10H', ip))
    while checksum >> 16: checksum = (checksum & 65535) + (checksum >> 16)
    assert checksum == 65535, 'IPv4 checksum'
    assert eth[:6] == b'\xff'*6
    assert ip[12:20] == bytes([192,168,0,2,192,168,0,3])
    sport,dport,ulen,cs = struct.unpack('!4H',eth[34:42])
    assert (sport,dport,ulen,cs) == (5000,6102,1304,0)
    payload = eth[42:]
    magic,fid,row,w,h,fmt = struct.unpack('!4sI4H',payload[:16])
    assert (magic,w,h,fmt)==(b'FPG1',640,4,1)
    assert len(payload)==1296
    assert (fid,row) not in seen
    seen.add((fid,row))
    words = struct.unpack('!640H',payload[16:])
    expected = tuple(((x%32)<<11)|((row+fid)<<5)|((x+row)%32) for x in range(640))
    assert words == expected, f'Pixel corruption at frame {fid} row {row}'
assert seen == {(f,y) for f in range(1,4) for y in range(4)}
print('PASS: every RGB565 pixel, frame/row, UDP lengths, IPv4 checksum and Ethernet FCS')
