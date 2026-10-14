"""Checked NMEA/GGA positions and UBX NAV-PVT receiver metadata."""

import re
import struct

NMEA = re.compile(rb"\$([A-Z]{2}[A-Z]{3}),([^\r\n]*?)\*([0-9A-Fa-f]{2})\r\n")


def nmea_records(data):
    valid = []
    invalid = 0
    for match in NMEA.finditer(data):
        body = match.group(1) + b"," + match.group(2)
        checksum = 0
        for byte in body:
            checksum ^= byte
        if checksum != int(match.group(3), 16):
            invalid += 1
            continue
        valid.append((match.group(1).decode(), match.group(2).decode().split(",")))
    return valid, invalid


def ubx_records(data):
    messages = []
    offset = 0
    while offset + 8 <= len(data):
        if data[offset : offset + 2] != b"\xb5\x62":
            offset += 1
            continue
        msg_class, msg_id, size = struct.unpack_from("<BBH", data, offset + 2)
        end = offset + 6 + size
        if end + 2 > len(data):
            offset += 1
            continue
        first = second = 0
        for byte in data[offset + 2 : end]:
            first = (first + byte) & 255
            second = (second + first) & 255
        if data[end : end + 2] != bytes((first, second)):
            offset += 1
            continue
        messages.append((msg_class, msg_id, data[offset + 6 : end]))
        offset = end + 2
    return messages


def degrees(value, hemisphere):
    digits = 2 if hemisphere in "NS" else 3
    result = int(value[:digits]) + float(value[digits:]) / 60
    return -result if hemisphere in "SW" else result


def read_log(data):
    nmea, bad = nmea_records(data)
    gga = []
    for name, fields in nmea:
        if not name.endswith("GGA") or len(fields) < 14:
            continue
        if not fields[1] or not fields[3]:
            continue
        gga.append(
            {
                "utc": fields[0],
                "latitude": degrees(fields[1], fields[2]),
                "longitude": degrees(fields[3], fields[4]),
                "quality": int(fields[5]),
                "satellites": int(fields[6]),
                "hdop": float(fields[7]),
            }
        )
    pvt = []
    for msg_class, msg_id, payload in ubx_records(data):
        if (msg_class, msg_id) == (1, 7) and len(payload) >= 92:
            pvt.append(
                {
                    "carrier": (payload[21] >> 6) & 3,
                    "differential": bool(payload[21] & 2),
                    "hacc_m": struct.unpack_from("<I", payload, 40)[0] / 1000,
                }
            )
    if len(gga) < 2:
        raise ValueError("At least two valid GGA positions are required")
    return nmea, bad, gga, pvt
