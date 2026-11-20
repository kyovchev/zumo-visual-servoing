import struct
import zlib
import cv2
import numpy as np


def mm_to_px(mm: float, dpi: int = 72) -> int:
    return round(mm * dpi / 25.4)


def set_png_dpi(png_bytes: bytes, dpi: int = 72) -> bytes:
    """Embed a PNG pHYs chunk with the specified DPI (pixels per inch)."""
    ppm = round(dpi / 0.0254)  # pixels per meter
    phys_data = struct.pack(">IIb", ppm, ppm, 1)  # unit=1 means meter
    crc = zlib.crc32(b"pHYs" + phys_data) & 0xFFFFFFFF
    phys_chunk = struct.pack(">I", len(phys_data)) + \
        b"pHYs" + phys_data + struct.pack(">I", crc)

    # Insert pHYs right after the 8-byte PNG signature + 25-byte IHDR chunk (offset 33)
    ihdr_end = 8 + 4 + 4 + 13 + 4  # 33 bytes
    return png_bytes[:ihdr_end] + phys_chunk + png_bytes[ihdr_end:]


def generate_aruco_printable(
    marker_id: int = 7,
    marker_mm: float = 50.0,
    white_border_mm: float = 5.0,
    black_border_px: int = 1,
    dpi: int = 72,
    output_path: str = "aruco_orig_id7_50mm_72dpi.png",
) -> None:
    # 1. Generate marker from Original ArUco dictionary (50 mm at 72 DPI -> 142 px)
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_ARUCO_ORIGINAL)
    marker_px = mm_to_px(marker_mm, dpi)
    marker_img = cv2.aruco.generateImageMarker(
        aruco_dict, marker_id, marker_px)

    # 2. Add 5 mm white border on all sides (14 px per side -> 170x170 px)
    white_border_px = mm_to_px(white_border_mm, dpi)
    canvas = cv2.copyMakeBorder(
        marker_img,
        top=white_border_px,
        bottom=white_border_px,
        left=white_border_px,
        right=white_border_px,
        borderType=cv2.BORDER_CONSTANT,
        value=255,
    )

    # 3. Add 1-pixel black border around the outside (172x172 px)
    final_img = cv2.copyMakeBorder(
        canvas,
        top=black_border_px,
        bottom=black_border_px,
        left=black_border_px,
        right=black_border_px,
        borderType=cv2.BORDER_CONSTANT,
        value=0,
    )

    # 4. Encode PNG with OpenCV and embed 72 DPI metadata
    ok, encoded = cv2.imencode(".png", final_img)
    if not ok:
        raise RuntimeError("Failed to encode PNG image.")

    png_with_dpi = set_png_dpi(encoded.tobytes(), dpi=dpi)
    with open(output_path, "wb") as f:
        f.write(png_with_dpi)

    print(
        f"Saved {output_path} ({final_img.shape[1]}x{final_img.shape[0]} px at {dpi} DPI)")


if __name__ == "__main__":
    generate_aruco_printable()
