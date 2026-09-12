"""Parameter schema consistency tests."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mdrive_core.schema.params import (  # noqa: E402
    GROUPS,
    PARAMS,
    READABLE_IDS,
    SET_IDS,
    assert_consistent,
    find_by_get_id,
)


class ParameterSchemaTests(unittest.TestCase):
    def test_schema_is_consistent(self) -> None:
        assert_consistent()

    def test_ids_are_unique_and_groups_are_populated(self) -> None:
        self.assertEqual(len(PARAMS), len(set(PARAMS)))
        self.assertTrue(GROUPS)
        self.assertTrue(all(definition.group in GROUPS for definition in PARAMS.values()))

    def test_rw_parameters_have_existing_get_entries(self) -> None:
        for set_id in SET_IDS:
            with self.subTest(set_id=set_id):
                definition = PARAMS[set_id]
                self.assertIsNotNone(definition.get_id)
                self.assertIn(definition.get_id, PARAMS)
                self.assertIs(find_by_get_id(definition.get_id), definition)

    def test_authoritative_ids_and_planned_commands_exist(self) -> None:
        spot_checks = {
            0x00,
            0x01,
            0x02,
            0x03,
            0x18,
            0x29,
            0x2D,
            0x2F,
            0x3F,
            0x41,
            0x4D,
            0x58,
            0x59,
            0x62,
            0x64,
            0x65,
            0x66,
            0x67,
            0x68,
            0x69,
            0x6A,
        }
        self.assertTrue(spot_checks <= PARAMS.keys())
        self.assertTrue({0x2D, 0x2F, 0x3F, 0x41, 0x59, 0x62} <= READABLE_IDS)


if __name__ == "__main__":
    unittest.main()
