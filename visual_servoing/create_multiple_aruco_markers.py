import struct
import zlib
from typing import List, Optional
import cv2
import numpy as np

PAPER_SIZES_MM = {
    "A4": (210.0, 297.0),
    "Letter": (215.9, 279.4),
}


def mm_to_px(mm: float, dpi: int = 72) -> int:
    return round(mm * dpi / 25.4)


def set_png_dpi(png_bytes: bytes, dpi: int = 72) -> bytes:
    """Embed a PNG pHYs chunk with the specified DPI (pixels per inch)."""
    ppm = round(dpi / 0.0254)
    phys_data = struct.pack(">IIb", ppm, ppm, 1)
    crc = zlib.crc32(b"pHYs" + phys_data) & 0xFFFFFFFF
    phys_chunk = struct.pack(">I", len(phys_data)) + \
        b"pHYs" + phys_data + struct.pack(">I", crc)

    ihdr_end = 8 + 4 + 4 + 13 + 4  # right after IHDR
    return png_bytes[:ihdr_end] + phys_chunk + png_bytes[ihdr_end:]


def build_marker_tile(
    aruco_dict,
    marker_id: int,
    marker_px: int,
    white_border_px: int,
    black_border_px: int = 1,
) -> np.ndarray:
    """Create a 50mm marker with a 5mm white border, small ID inside it, and 1-px black border."""
    marker_img = cv2.aruco.generateImageMarker(
        aruco_dict, marker_id, marker_px)

    canvas = cv2.copyMakeBorder(
        marker_img,
        top=white_border_px,
        bottom=white_border_px,
        left=white_border_px,
        right=white_border_px,
        borderType=cv2.BORDER_CONSTANT,
        value=255,
    )

    # Place tiny ID inside the bottom white border, close to the outer edge
    label = str(marker_id)
    font = cv2.FONT_HERSHEY_PLAIN
    font_scale = 0.65
    thickness = 1
    (text_w, text_h), _ = cv2.getTextSize(label, font, font_scale, thickness)

    if text_h <= white_border_px - 2 and text_w <= canvas.shape[1] - 4:
        text_x = (canvas.shape[1] - text_w) // 2
        text_y = canvas.shape[0] - 2
        cv2.putText(
            canvas,
            label,
            (text_x, text_y),
            font,
            font_scale,
            0,
            thickness,
            lineType=cv2.LINE_AA,
        )

    tile = cv2.copyMakeBorder(
        canvas,
        top=black_border_px,
        bottom=black_border_px,
        left=black_border_px,
        right=black_border_px,
        borderType=cv2.BORDER_CONSTANT,
        value=0,
    )
    return tile


def generate_aruco_sheet(
    marker_ids: Optional[List[int]] = None,
    count: int = 12,
    start_id: int = 0,
    paper: str = "A4",
    marker_mm: float = 50.0,
    white_border_mm: float = 5.0,
    black_border_px: int = 1,
    margin_mm: float = 10.0,
    gap_mm: float = 4.0,
    dpi: int = 72,
    output_path: str = "aruco_sheet_a4_72dpi.png",
) -> None:
    if paper not in PAPER_SIZES_MM:
        raise ValueError(
            f"Unsupported paper '{paper}'. Choose from {list(PAPER_SIZES_MM.keys())}.")

    page_w_mm, page_h_mm = PAPER_SIZES_MM[paper]
    page_w_px = mm_to_px(page_w_mm, dpi)
    page_h_px = mm_to_px(page_h_mm, dpi)

    marker_px = mm_to_px(marker_mm, dpi)
    white_border_px = mm_to_px(white_border_mm, dpi)
    margin_px = mm_to_px(margin_mm, dpi)
    gap_px = mm_to_px(gap_mm, dpi)

    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_ARUCO_ORIGINAL)
    sample_tile = build_marker_tile(
        aruco_dict, 0, marker_px, white_border_px, black_border_px
    )
    tile_h, tile_w = sample_tile.shape[:2]

    usable_w = page_w_px - 2 * margin_px
    usable_h = page_h_px - 2 * margin_px

    cols = max(1, (usable_w + gap_px) // (tile_w + gap_px))
    rows = max(1, (usable_h + gap_px) // (tile_h + gap_px))
    capacity = cols * rows

    if marker_ids is None:
        n = min(count, capacity)
        marker_ids = list(range(start_id, start_id + n))
    elif len(marker_ids) > capacity:
        raise ValueError(
            f"{len(marker_ids)} markers exceed {paper} capacity ({capacity} markers in a {cols}x{rows} grid)."
        )

    page = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)

    grid_w = cols * tile_w + max(0, cols - 1) * gap_px
    grid_h = rows * tile_h + max(0, rows - 1) * gap_px
    start_x = margin_px + (usable_w - grid_w) // 2
    start_y = margin_px + (usable_h - grid_h) // 2

    for idx, mid in enumerate(marker_ids):
        r = idx // cols
        c = idx % cols
        tile = build_marker_tile(
            aruco_dict, mid, marker_px, white_border_px, black_border_px
        )
        x0 = start_x + c * (tile_w + gap_px)
        y0 = start_y + r * (tile_h + gap_px)
        page[y0: y0 + tile_h, x0: x0 + tile_w] = tile

    ok, encoded = cv2.imencode(".png", page)
    if not ok:
        raise RuntimeError("Failed to encode PNG sheet.")

    png_with_dpi = set_png_dpi(encoded.tobytes(), dpi=dpi)
    with open(output_path, "wb") as f:
        f.write(png_with_dpi)

    print(
        f"Saved {output_path} ({page_w_px}x{page_h_px} px, {paper} at {dpi} DPI) "
        f"with IDs {marker_ids} in a {cols}x{rows} grid."
    )


if __name__ == "__main__":
    generate_aruco_sheet(
        count=12,
        start_id=0,
        paper="A4",
        margin_mm=5.0,
        gap_mm=3.0,
        output_path="aruco_0_to_11_a4_72dpi.png",
    )
