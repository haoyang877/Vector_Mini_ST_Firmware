import math
import unittest

from friction_identification import Sample, constrained_fit, parse_status


class FrictionIdentificationToolTests(unittest.TestCase):
    def test_bidirectional_fit(self) -> None:
        speeds = (0.1, 0.2, 0.4, 0.8)
        samples = []
        for speed in speeds:
            omega = speed * 2.0 * math.pi
            samples.append(Sample(len(samples), speed, speed, 0.12 + 0.03 * omega, 2000))
            samples.append(Sample(len(samples), -speed, -speed, -0.15 - 0.02 * omega, 2000))

        positive = constrained_fit(samples, True)
        negative = constrained_fit(samples, False)

        self.assertAlmostEqual(positive.coulomb_a, 0.12, places=7)
        self.assertAlmostEqual(positive.viscous_a_per_rad_s, 0.03, places=7)
        self.assertAlmostEqual(negative.coulomb_a, 0.15, places=7)
        self.assertAlmostEqual(negative.viscous_a_per_rad_s, 0.02, places=7)
        self.assertAlmostEqual(positive.rmse_a, 0.0, places=7)
        self.assertAlmostEqual(negative.rmse_a, 0.0, places=7)

    def test_negative_slope_is_constrained(self) -> None:
        samples = [
            Sample(index, speed, speed, 0.2 - 0.01 * speed * 2.0 * math.pi, 1000)
            for index, speed in enumerate((0.1, 0.2, 0.4, 0.8))
        ]
        fit = constrained_fit(samples, True)
        self.assertEqual(fit.viscous_a_per_rad_s, 0.0)
        self.assertGreater(fit.coulomb_a, 0.0)

    def test_status_parser(self) -> None:
        status = parse_status(
            "friction_state=2,reason=0,point=3,progress=37.5,candidate=0"
        )
        self.assertEqual(status["friction_state"], 2.0)
        self.assertEqual(status["point"], 3.0)
        self.assertEqual(status["progress"], 37.5)


if __name__ == "__main__":
    unittest.main()
