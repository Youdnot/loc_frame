"""Image bytes conversion utilities.

This module provides functions to convert image bytes to OpenCV-compatible formats.
"""

import numpy as np
import cv2


def decode_bytes_to_cv2(image_bytes):
    """Decodes raw image bytes into an OpenCV image (numpy array).

    Uses OpenCV to decode the byte stream directly into a BGR numpy array.

    Args:
        image_bytes: The raw bytes of the image (e.g., from a network stream).

    Returns:
        A numpy array representing the image in BGR format if successful,
        otherwise None.
    """
    try:
        # Convert bytes to a numpy array of uint8. This is a fast operation as
        # it creates a memory view.
        nparr = np.frombuffer(image_bytes, np.uint8)

        # Decode the image array. cv2.IMREAD_COLOR loads the image in BGR format.
        # Use cv2.IMREAD_GRAYSCALE if only grayscale is needed for SLAM.
        img_cv2 = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        return img_cv2
    except (ValueError, cv2.error) as e:
        print(f"OpenCV Decode Error: {e}")
        return None
    except Exception as e:
        print(f"Unexpected Error during image decoding: {e}")
        return None