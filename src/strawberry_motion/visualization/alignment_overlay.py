"""Image overlays used while aligning the overview camera to the workspace."""

from dataclasses import dataclass
from typing import Tuple

import cv2
import numpy as np


Color = Tuple[int, int, int]


@dataclass(frozen=True)
class CrosshairStyle:
    """Rendering options for an alignment crosshair on a BGR image."""

    crosshair_length_px: int = 60
    line_thickness_px: int = 2
    guide_margin_ratio: float = 0.08
    axis_color_bgr: Color = (0, 180, 0)
    crosshair_color_bgr: Color = (0, 255, 255)
    guide_color_bgr: Color = (255, 180, 0)
    text_color_bgr: Color = (0, 255, 255)


def draw_alignment_overlay(
    image_bgr: np.ndarray, style: CrosshairStyle = CrosshairStyle()
) -> np.ndarray:
    """Return an image with center axes and overview alignment guides."""
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("alignment overlay requires a BGR image with three channels")
    if image_bgr.dtype != np.uint8:
        raise ValueError("alignment overlay requires an uint8 image")

    height, width = image_bgr.shape[:2]
    if height < 2 or width < 2:
        raise ValueError("alignment overlay requires an image larger than 1 pixel")

    overlay = image_bgr.copy()
    center_x = width // 2
    center_y = height // 2
    length = max(1, min(style.crosshair_length_px, min(width, height) // 2))
    thickness = max(1, style.line_thickness_px)

    # Full axes help align the long tape boundaries; the highlighted center marks the origin.
    cv2.line(overlay, (center_x, 0), (center_x, height - 1), style.axis_color_bgr, 1)
    cv2.line(overlay, (0, center_y), (width - 1, center_y), style.axis_color_bgr, 1)
    cv2.line(
        overlay,
        (center_x - length, center_y),
        (center_x + length, center_y),
        style.crosshair_color_bgr,
        thickness,
    )
    cv2.line(
        overlay,
        (center_x, center_y - length),
        (center_x, center_y + length),
        style.crosshair_color_bgr,
        thickness,
    )
    cv2.circle(overlay, (center_x, center_y), max(4, thickness + 2), style.crosshair_color_bgr, 1)

    margin = int(min(width, height) * max(0.0, min(style.guide_margin_ratio, 0.45)))
    if margin > 0:
        cv2.rectangle(
            overlay,
            (margin, margin),
            (width - margin - 1, height - margin - 1),
            style.guide_color_bgr,
            1,
        )

    cv2.putText(
        overlay,
        "ALIGN TAPE CROSSING TO CENTER (%d, %d)" % (center_x, center_y),
        (12, max(24, margin - 10 if margin > 35 else 24)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        style.text_color_bgr,
        2,
        cv2.LINE_AA,
    )
    return overlay
