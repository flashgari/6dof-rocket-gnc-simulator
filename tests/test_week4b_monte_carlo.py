import unittest

from scripts.run_week4b_monte_carlo import run_case, sample_trial, summary
import random


class Week4BMonteCarloTests(unittest.TestCase):
    def test_sampling_is_reproducible_with_fixed_seed(self):
        a = sample_trial(random.Random(4242), 0)
        b = sample_trial(random.Random(4242), 0)

        self.assertEqual(a, b)

    def test_lqr_and_pd_outperform_open_loop_on_small_campaign(self):
        rng = random.Random(4242)
        rows = []
        for trial in range(8):
            params = sample_trial(rng, trial)
            for controller in ("open_loop", "pd_tvc", "lqr_tvc"):
                rows.append(run_case(controller, params))

        data = summary(rows)

        self.assertGreater(data["pd_tvc"]["success_rate"], data["open_loop"]["success_rate"])
        self.assertGreater(data["lqr_tvc"]["success_rate"], data["open_loop"]["success_rate"])
        self.assertLessEqual(data["lqr_tvc"]["median_max_lateral_m"], data["pd_tvc"]["median_max_lateral_m"])


if __name__ == "__main__":
    unittest.main()
