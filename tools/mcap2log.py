"""Convert a vioh MCAP recording to the flat log that the C++ tools read.

Development aid only: it never runs on the aircraft.
Usage: python mcap2log.py in.mcap out.vlog      (needs: pip install mcap lz4)

Flat log = a sequence of records, little-endian:
  'I' int64 t_ns  float32 gx gy gz ax ay az       IMU, rad/s and m/s^2
  'F' int64 t_ns  uint16 width  uint16 height  width*height bytes   mono8 frame
"""
import struct
import sys

from mcap.reader import make_reader


def fields(buf):
    """Minimal protobuf decode: {field number: last value}. Varints as int, the rest as bytes."""
    out, pos = {}, 0

    def varint():
        nonlocal pos
        value, shift = 0, 0
        while True:
            byte = buf[pos]
            pos += 1
            value |= (byte & 127) << shift
            shift += 7
            if byte < 128:
                return value

    while pos < len(buf):
        key = varint()
        number, wire = key >> 3, key & 7
        if wire == 0:
            out[number] = varint()
        elif wire == 1:
            out[number] = buf[pos:pos + 8]
            pos += 8
        elif wire == 5:
            out[number] = buf[pos:pos + 4]
            pos += 4
        elif wire == 2:
            size = varint()
            out[number] = buf[pos:pos + size]
            pos += size
        else:
            raise ValueError(f"unsupported protobuf wire type {wire}")
    return out


def main(src, dst):
    counts = {"I": 0, "F": 0}
    with open(src, "rb") as fin, open(dst, "wb") as fout:
        for _schema, channel, message in make_reader(fin).iter_messages(topics=["/imu", "/cam/nadir"]):
            msg = fields(message.data)
            if channel.topic == "/imu":
                # vioh.v1.Imu: 1 t_ns (varint), 2..7 gx gy gz ax ay az (double). proto3 omits zeros.
                six = [struct.unpack("<d", msg[n])[0] if n in msg else 0.0 for n in range(2, 8)]
                fout.write(b"I" + struct.pack("<q6f", msg[1], *six))
                counts["I"] += 1
            else:
                # foxglove.RawImage: 1 timestamp, 2 width, 3 height, 4 encoding, 5 step, 6 data
                stamp = fields(msg[1])
                t_ns = stamp.get(1, 0) * 1_000_000_000 + stamp.get(2, 0)
                width, height = struct.unpack("<I", msg[2])[0], struct.unpack("<I", msg[3])[0]
                if msg[4] != b"mono8" or len(msg[6]) != width * height:
                    raise ValueError(f"unexpected image: {msg[4]!r} {width}x{height} {len(msg[6])} bytes")
                fout.write(b"F" + struct.pack("<qHH", t_ns, width, height) + msg[6])
                counts["F"] += 1
    print(f"wrote {dst}: {counts['I']} imu samples, {counts['F']} frames")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
