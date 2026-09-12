"""Offline tests for the synthetic Loader fault-image generator; never opens hardware."""

from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "loader" / "tools"))

from fault_images import APP_BASE, IMAGE_SIZE, VALID_SP, build_all  # noqa: E402
from loader_proto import crc32  # noqa: E402


class FaultImageTests(unittest.TestCase):
    def setUp(self) -> None:
        self.images = {image.name: image for image in build_all()}

    def _words(self, name: str) -> tuple[int, int]:
        data = self.images[name].data
        return struct.unpack_from("<I", data, 0)[0], struct.unpack_from("<I", data, 4)[0]

    def test_all_expected_variants_exist(self) -> None:
        self.assertEqual(
            set(self.images),
            {"hang_no_comms", "return_from_reset", "bad_sp_out_of_sram", "bad_sp_unaligned",
             "bad_reset_thumb_bit", "bad_reset_outside"},
        )

    def test_every_image_is_programmable(self) -> None:
        for image in self.images.values():
            with self.subTest(image=image.name):
                self.assertEqual(image.size, IMAGE_SIZE)
                self.assertEqual(image.size % 8, 0)
                self.assertLessEqual(image.size, 0x18000)

    def test_valid_variants_satisfy_every_structural_check(self) -> None:
        for name in ("hang_no_comms", "return_from_reset"):
            with self.subTest(image=name):
                sp, reset = self._words(name)
                self.assertEqual(sp & 7, 0)
                self.assertLessEqual(0x20000020, sp)
                self.assertLessEqual(sp, 0x20008000)
                self.assertEqual(reset & 1, 1)
                self.assertLessEqual(APP_BASE, reset & ~1)
                self.assertLess(reset & ~1, APP_BASE + self.images[name].size)

    def test_bad_sp_out_of_sram_fails_only_the_range_check(self) -> None:
        sp, reset = self._words("bad_sp_out_of_sram")
        self.assertFalse(0x20000020 <= sp <= 0x20008000)
        self.assertEqual(reset & 1, 1)

    def test_bad_sp_unaligned_fails_only_the_alignment_check(self) -> None:
        sp, reset = self._words("bad_sp_unaligned")
        self.assertNotEqual(sp & 7, 0)
        self.assertTrue(0x20000020 <= sp <= 0x20008000)
        self.assertEqual(reset & 1, 1)

    def test_bad_reset_thumb_bit_fails_only_the_thumb_check(self) -> None:
        _, reset = self._words("bad_reset_thumb_bit")
        self.assertEqual(reset & 1, 0)
        self.assertEqual(reset & ~1, APP_BASE + 0x100)

    def test_bad_reset_outside_fails_only_the_range_check(self) -> None:
        _, reset = self._words("bad_reset_outside")
        self.assertEqual(reset & 1, 1)
        self.assertGreaterEqual(reset & ~1, APP_BASE + self.images["bad_reset_outside"].size)

    def test_declared_boot_validity_matches_structural_rules(self) -> None:
        for image in self.images.values():
            with self.subTest(image=image.name):
                sp, reset = self._words(image.name)
                valid = (
                    0x20000020 <= sp <= 0x20008000
                    and sp & 7 == 0
                    and reset & 1 == 1
                    and APP_BASE <= reset & ~1 < APP_BASE + image.size
                )
                self.assertEqual(valid, image.boot_valid)

    def test_every_handler_points_into_the_image_with_thumb_bit(self) -> None:
        for image in self.images.values():
            with self.subTest(image=image.name):
                for index in range(2, 64):
                    handler = struct.unpack_from("<I", image.data, index * 4)[0]
                    self.assertEqual(handler & 1, 1)
                    self.assertTrue(APP_BASE <= handler & ~1 < APP_BASE + image.size)

    def test_crc_tracks_content_and_is_deterministic(self) -> None:
        first = build_all()
        second = build_all()
        self.assertEqual([image.data for image in first], [image.data for image in second])
        self.assertEqual(len({image.crc32 for image in first}), len(first))
        self.assertEqual(self.images["hang_no_comms"].crc32, crc32(self.images["hang_no_comms"].data))
        self.assertEqual(VALID_SP, struct.unpack_from("<I", self.images["hang_no_comms"].data, 0)[0])


if __name__ == "__main__":
    unittest.main()
