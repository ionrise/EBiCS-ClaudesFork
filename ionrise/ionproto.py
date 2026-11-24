"""IONRISE protocol reference implementation in Python (framing, CRC, handshake proof).

Mirrors protocol/ionproto.c. Used for test vectors in IONRISE_PROTOCOL.md and by PC tools
(sniffer, setup app). Run `python protocol/ionproto.py` to print the self-test and examples.
"""
import struct

SOF = 0xA5
MAX_PAYLOAD = 64
RESP = 0x80
KEY = b"IONRISE"

CMD = {"HELLO": 0x01, "AUTH": 0x02, "TELEMETRY": 0x10, "SET_READ": 0x20, "SET_WRITE": 0x21,
       "SET_APPLY": 0x22, "ACTION": 0x30, "DIAG": 0x31}


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(cmd: int, seq: int, payload: bytes = b"") -> bytes:
    assert len(payload) <= MAX_PAYLOAD
    body = bytes([len(payload) + 2, cmd, seq]) + payload
    return bytes([SOF]) + body + struct.pack("<H", crc16(body))


def proof(nonce: int, ident: int) -> int:
    return crc16(KEY, crc16(struct.pack("<IB", nonce, ident)))


class Receiver:
    """Byte-at-a-time receiver with resync, same behaviour as ion_rx_byte()."""

    def __init__(self):
        self.buf = bytearray()

    def _shift(self):
        i = 1
        while i < len(self.buf) and self.buf[i] != SOF:
            i += 1
        del self.buf[:i]

    def feed(self, b: int):
        """Returns (cmd, seq, payload) when a frame completes, else None."""
        if not self.buf and b != SOF:
            return None
        self.buf.append(b)
        while len(self.buf) >= 2:
            n = self.buf[1]
            if n < 2 or n > MAX_PAYLOAD + 2:
                self._shift()
                continue
            if len(self.buf) < n + 4:
                return None
            c = crc16(bytes(self.buf[1:n + 2]))
            if self.buf[n + 2] == (c & 0xFF) and self.buf[n + 3] == (c >> 8):
                f = bytes(self.buf)
                self.buf.clear()
                return f[2], f[3], f[4:n + 2]
            self._shift()
        return None


def _hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


if __name__ == "__main__":
    assert crc16(b"123456789") == 0x29B1, "CRC check value"
    # round trip with noise and a false SOF in front
    f = build(0x10, 7, bytes([153, 3, 5, 1, 0, 0, 100, 0]))
    r = Receiver()
    got = [x for x in (r.feed(b) for b in bytes([0x00, 0xA5, 0x05, 0x99]) + f) if x]
    assert got == [(0x10, 7, bytes([153, 3, 5, 1, 0, 0, 100, 0]))], got
    print("self-test OK (CRC check 0x29B1, round trip with resync)")

    nd, nc = 0x12345678, 0x9ABCDEF0
    print("HELLO req :", _hex(build(0x01, 1, bytes([1, 0, 1, 0]) + struct.pack("<IH", nd, 0x0100))))
    hello_resp = bytes([1, 0, 1, 0]) + struct.pack("<HBBHBBHHI", 0x0100, 1, 1, 256, 1, 0,
                                                   0x0207, proof(nd, 1), nc)
    print("HELLO resp:", _hex(build(0x81, 1, hello_resp)), f"(proof_c = {proof(nd, 1):04X})")
    print("AUTH req  :", _hex(build(0x02, 2, struct.pack("<HB", proof(nc, 1), 1))), f"(proof_d = {proof(nc, 1):04X})")
    print("AUTH resp :", _hex(build(0x82, 2, bytes([0]))))
    print("TELEM req :", _hex(build(0x10, 3, bytes([153, 3, 5, 0x01, 0, 0, 100, 0]))))
    tr = struct.pack("<HHhhbbBBHBBHHBB", 2534, 5062, 123, 250, 41, 35, 72, 255, 0xFFFF, 150, 48,
                     0x100D, 0, 0, 0)
    print("TELEM resp:", _hex(build(0x90, 3, tr)), "(25.34 km/h, 50.62 V, 12.3 A)")
    print("SET_READ  :", _hex(build(0x20, 4, bytes([0]) + struct.pack("<HB", 60, 8))), "(active, speed_limit + wheel)")
