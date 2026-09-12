"""Application image preparation for CAN upgrades (no hardware access)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import zlib

MAX_IMAGE_BYTES = 0x18000  # 96 KiB application partition


@dataclass(frozen=True)
class PreparedImage:
    """Trimmed, 8-byte aligned image ready for BEGIN/PROGRAM."""

    data: bytes
    size: int
    crc32: int

    def to_dict(self) -> dict[str, int]:
        return {"size": self.size, "crc32": self.crc32}


class ImagePrepError(ValueError):
    """The image file is empty, too large, or unreadable."""


def prepare_image(data: bytes, max_bytes: int = MAX_IMAGE_BYTES) -> PreparedImage:
    """Strip trailing 0xFF, pad to 8 bytes, and compute the upgrade CRC32."""
    if not data:
        raise ImagePrepError("image is empty")
    end = len(data)
    while end > 0 and data[end - 1] == 0xFF:
        end -= 1
    size = (end + 7) // 8 * 8
    if size == 0:
        raise ImagePrepError("image is all 0xFF (erased)")
    if size > max_bytes:
        raise ImagePrepError(f"image {size} B exceeds partition limit {max_bytes} B")
    image = data[:size]
    if len(image) < size:
        image = image + b"\xFF" * (size - len(image))
    return PreparedImage(data=image, size=size, crc32=zlib.crc32(image) & 0xFFFFFFFF)


def prepare_image_file(path: Path, max_bytes: int = MAX_IMAGE_BYTES) -> PreparedImage:
    """Read and prepare an image file from disk."""
    try:
        data = Path(path).read_bytes()
    except OSError as exc:
        raise ImagePrepError(f"cannot read image: {exc}") from exc
    return prepare_image(data, max_bytes)
