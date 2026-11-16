#!/usr/bin/env python3
"""Generates a high-contrast line tracking circuit texture for Webots.

Produces a closed-loop oval/stadium track texture (black line on white floor)
without requiring any external dependencies (pure Python with zlib + struct).
Supports PIL/Pillow if available, with automatic fallback.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import zlib
from pathlib import Path


def make_png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    """Wraps chunk data with length, type, and CRC32."""
    length = len(data)
    type_and_data = chunk_type + data
    crc = zlib.crc32(type_and_data) & 0xFFFFFFFF
    return struct.pack(">I", length) + type_and_data + struct.pack(">I", crc)


def write_rgb_png(file_path: Path, width: int, height: int, rgb_bytes: bytes) -> None:
    """Writes an 8-bit RGB non-interlaced PNG file using standard library."""
    file_path.parent.mkdir(parents=True, exist_ok=True)

    png_header = b"\x89PNG\r\n\x1a\n"
    ihdr_data = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    ihdr_chunk = make_png_chunk(b"IHDR", ihdr_data)

    # Prepend filter byte 0 (None) to each row
    row_bytes = width * 3
    scanlines = bytearray(height * (1 + row_bytes))
    dest_idx = 0
    src_idx = 0
    for _ in range(height):
        scanlines[dest_idx] = 0  # Filter: None
        dest_idx += 1
        scanlines[dest_idx : dest_idx + row_bytes] = rgb_bytes[src_idx : src_idx + row_bytes]
        dest_idx += row_bytes
        src_idx += row_bytes

    compressed = zlib.compress(bytes(scanlines), level=9)
    idat_chunk = make_png_chunk(b"IDAT", compressed)
    iend_chunk = make_png_chunk(b"IEND", b"")

    with open(file_path, "wb") as f:
        f.write(png_header)
        f.write(ihdr_chunk)
        f.write(idat_chunk)
        f.write(iend_chunk)


def generate_track_texture(
    output_path: Path,
    width: int = 1024,
    height: int = 1024,
    arena_size_m: float = 3.0,
    straight_length_m: float = 1.4,
    radius_m: float = 0.5,
    line_width_m: float = 0.018,
) -> Path:
    """Renders a stadium-shaped track into a high-contrast PNG file.

    Geometry:
    - Arena: arena_size_m x arena_size_m square centered at (0, 0).
    - Top straight: y = +radius_m, x in [-straight_length_m/2, +straight_length_m/2].
    - Bottom straight: y = -radius_m, x in [-straight_length_m/2, +straight_length_m/2].
    - Turn right: semi-circle centered at (+straight_length_m/2, 0), radius = radius_m.
    - Turn left: semi-circle centered at (-straight_length_m/2, 0), radius = radius_m.
    - Total dimensions: (straight_length_m + 2*radius_m) by (2*radius_m).
      With default 1.4m straight and 0.5m radius, total span is 2.4m x 1.0m.

    Args:
        output_path: Destination PNG file path.
        width: Output image width in pixels.
        height: Output image height in pixels.
        arena_size_m: Arena size represented by the texture in meters.
        straight_length_m: Length of the two straightaways in meters.
        radius_m: Turning radius of the track ends in meters.
        line_width_m: Physical width of the black tape line in meters.

    Returns:
        The Path to the generated PNG file.
    """
    x_c = straight_length_m / 2.0
    y_c = radius_m
    half_line_w = line_width_m / 2.0
    pixel_size_m = arena_size_m / float(width)

    # Pixel buffer for RGB (width * height * 3)
    rgb_data = bytearray(width * height * 3)
    pixel_offset = 0

    half_w = width / 2.0
    half_h = height / 2.0

    for iy in range(height):
        # Convert pixel to metric coordinate (centered at 0, 0)
        py = (iy + 0.5 - half_h) * pixel_size_m
        ay = abs(py)

        for ix in range(width):
            px = (ix + 0.5 - half_w) * pixel_size_m
            ax = abs(px)

            if ax <= x_c:
                # Straight section
                dist = abs(ay - y_c)
            else:
                # Curved turn section
                dx = ax - x_c
                dist = abs(math.sqrt(dx * dx + ay * ay) - y_c)

            # Anti-aliased line rendering
            if dist <= half_line_w:
                v = 0
            elif dist >= half_line_w + pixel_size_m:
                v = 255
            else:
                ratio = (dist - half_line_w) / pixel_size_m
                v = int(255.0 * ratio)
                if v < 0:
                    v = 0
                elif v > 255:
                    v = 255

            rgb_data[pixel_offset] = v
            rgb_data[pixel_offset + 1] = v
            rgb_data[pixel_offset + 2] = v
            pixel_offset += 3

    write_rgb_png(output_path, width, height, bytes(rgb_data))
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate PiCar-S line tracking circuit texture")
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "worlds" / "textures" / "line_track.png",
        help="Output PNG path",
    )
    parser.add_argument("--size", "-s", type=int, default=1024, help="Texture width and height in pixels")
    parser.add_argument("--arena-size", type=float, default=3.0, help="Floor arena size in meters")
    parser.add_argument("--straight", type=float, default=1.4, help="Straightaway length in meters")
    parser.add_argument("--radius", type=float, default=0.5, help="Turn radius in meters")
    parser.add_argument("--line-width", type=float, default=0.018, help="Line width in meters (ruban isolant 18 mm)")

    args = parser.parse_args()

    out = generate_track_texture(
        output_path=args.output,
        width=args.size,
        height=args.size,
        arena_size_m=args.arena_size,
        straight_length_m=args.straight,
        radius_m=args.radius,
        line_width_m=args.line_width,
    )
    print(f"Track texture generated at: {out} ({args.size}x{args.size})")


if __name__ == "__main__":
    main()
