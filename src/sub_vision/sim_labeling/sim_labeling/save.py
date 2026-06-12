"""
Save labeled images and annotations in YOLOv11 segmentation format.

YOLOv11 segmentation format (one line per object instance):
    <class_id> <x1> <y1> <x2> <y2> ... <xN> <yN>

All coordinates normalized to [0, 1] relative to image dimensions.
This uses the segmentation camera's pixel-perfect silhouettes directly,
which is more accurate than bounding box detection.
"""

import cv2
import numpy as np
import time
import os
from pathlib import Path

# Minimum contour area (in pixels) to keep an annotation.
# Discard very small detections that are likely noise or distant objects.
MIN_CONTOUR_AREA_PX = 400  # ~20x20 pixels minimum

# Contour approximation factor: fraction of arc length used as epsilon
# for cv2.approxPolyDP.  Lower = more points = tighter fit.
APPROX_EPSILON_FACTOR = 0.02


def compute_segmentation_polygons(
    seg_img: np.ndarray,
    pixel_to_class: dict[int, int],
    min_contour_area_px: int = MIN_CONTOUR_AREA_PX,
) -> list[tuple[int, list[tuple[float, float]]]]:
    """
    Compute YOLO segmentation polygons from a segmentation image.

    Each unique segmentation pixel value is treated independently so that
    physically separate parts of the same class (e.g. the individual pipes
    of the gate) produce their own polygon annotation rather than being
    merged into one giant concave outline.

    Args:
        seg_img:        (H, W) uint16 segmentation image where pixel value
                        = objectId + 1 (0 = background).
        pixel_to_class: Mapping from segmentation pixel value to YOLO
                        class_id.

    Returns:
        List of (class_id, [(x1, y1), (x2, y2), ...]) tuples,
        with coordinates normalized to [0, 1].
    """
    height, width = seg_img.shape[:2]

    # Extract contours per individual pixel value (mesh part) rather than
    # merging by class.  This keeps disconnected parts as separate
    # annotations and avoids huge concave polygons that span across
    # background.
    annotations: list[tuple[int, list[tuple[float, float]]]] = []
    unique_vals = np.unique(seg_img)

    for pv in unique_vals:
        pv_int = int(pv)
        if pv_int == 0:
            continue  # background
        class_id = pixel_to_class.get(pv_int, -1)
        if class_id < 0:
            continue  # not a labeled prop

        mask = (seg_img == pv_int).astype(np.uint8)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            # Skip tiny contours
            if cv2.contourArea(cnt) < min_contour_area_px:
                continue

            # Approximate the contour to reduce point count
            epsilon = APPROX_EPSILON_FACTOR * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            approx = approx.reshape(-1, 2)  # (N,1,2) -> (N,2)

            # Need at least 3 points for a valid polygon
            if len(approx) < 3:
                continue

            # Normalize coordinates to [0, 1]
            points = [(float(x) / width, float(y) / height) for x, y in approx]
            annotations.append((class_id, points))

    return annotations


def save_annotation(
    annotations: list[tuple[int, list[tuple[float, float]]]],
    file_path: str,
) -> None:
    """
    Save annotations in YOLOv11 segmentation format.

    Each line: <class_id> <x1> <y1> <x2> <y2> ... <xN> <yN>
    """
    with open(file_path, "w") as f:
        for class_id, points in annotations:
            coords = " ".join(f"{x:.6f} {y:.6f}" for x, y in points)
            f.write(f"{class_id} {coords}\n")


def save_labeled_image(
    front_cam_img: np.ndarray,
    seg_img: np.ndarray,
    pixel_to_class: dict[int, int],
    output_dir: str = "train_imgs",
    class_names: list[str] | None = None,
    min_contour_area_px: int = MIN_CONTOUR_AREA_PX,
) -> bool:
    """
    Save a training image and its YOLO annotation.

    Args:
        front_cam_img:  The RGB camera image (H, W, 3).
        seg_img:        The segmentation image (H, W) uint16.
        pixel_to_class: Mapping from segmentation pixel value to class_id.
        output_dir:     Directory to save images and labels.
        class_names:    Optional list of class names for logging.

    Returns:
        True if an image was saved (had valid annotations), False otherwise.
    """
    annotations = compute_segmentation_polygons(
        seg_img,
        pixel_to_class,
        min_contour_area_px=min_contour_area_px,
    )

    if not annotations:
        return False

    # Create output directories
    img_dir = Path(output_dir) / "images"
    lbl_dir = Path(output_dir) / "labels"
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    timestamp = f"{time.time():.6f}"
    base_name = f"seg_{timestamp}"

    img_path = img_dir / f"{base_name}.png"
    lbl_path = lbl_dir / f"{base_name}.txt"

    # cv_bridge decodes rgb8 as RGB, but cv2.imwrite expects BGR
    bgr_img = cv2.cvtColor(front_cam_img, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(img_path), bgr_img)
    save_annotation(annotations, str(lbl_path))

    return True


def save_dataset_yaml(
    class_names: list[str],
    output_dir: str = "train_imgs",
) -> None:
    """
    Save the dataset YAML file for YOLOv11 training.
    """
    yaml_path = Path(output_dir) / "dataset.yaml"

    # Only write if it doesn't exist yet
    if yaml_path.exists():
        return

    yaml_path.parent.mkdir(parents=True, exist_ok=True)

    with open(yaml_path, "w") as f:
        f.write(f"path: {Path(output_dir).resolve()}\n")
        f.write("train: images\n")
        f.write("val: images\n")
        f.write(f"nc: {len(class_names)}\n")
        f.write(f"names: {class_names}\n")
