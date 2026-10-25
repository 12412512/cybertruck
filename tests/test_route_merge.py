"""连续段组合的全局选择；几何与闭环仍由正式场景回归验证。"""

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi'))
from robot.nav.frames import Pose
from robot.nav.planner import Planner


def segment(first, last, seconds):
    reference = SimpleNamespace(start=Pose(first, 0., 0.), end=Pose(last, 0., 0.))
    return SimpleNamespace(reference=reference, cost_s=seconds, road=f'road{first}', kind='LINE', corner=None)


class MergeTests(unittest.TestCase):
    def planner(self, candidates):
        planner = Planner.__new__(Planner)
        planner.cancelled = lambda: False
        planner.lookahead = True
        planner.make = lambda kind, start, end, *args, **kwargs: (
            candidates.get((start.x_m, end.x_m)), [])
        return planner

    def test_two_merges_beat_largest_immediate_gain(self):
        chain = [segment(index, index + 1, 5.) for index in range(4)]
        candidates = {(0, 2): segment(0, 2, 4.), (0, 3): segment(0, 3, 8.),
                      (2, 4): segment(2, 4, 4.)}
        result = self.planner(candidates).shortcut(chain, ())
        self.assertEqual(sum(item.cost_s for item in result), 8.)
        self.assertEqual([(item.reference.start.x_m, item.reference.end.x_m) for item in result],
                         [(0, 2), (2, 4)])

    def test_unavailable_or_slower_shortcut_keeps_original(self):
        chain = [segment(0, 1, 2.), segment(1, 2, 2.)]
        self.assertEqual(self.planner({}).shortcut(chain, ()), chain)
        self.assertEqual(self.planner({(0, 2): segment(0, 2, 5.)}).shortcut(chain, ()), chain)

    def test_cancel_stops_search(self):
        planner = self.planner({})
        planner.cancelled = lambda: True
        with self.assertRaises(InterruptedError):
            planner.shortcut([segment(0, 1, 2.)], ())

    def test_lead_in_keeps_existing_turn_window(self):
        lead = segment(0, 1, 3.)
        window = segment(1, 3, 8.)
        window.kind = 'WINDOW'
        window.corner = Pose(2., 0., 0.)
        window.reference.spec = SimpleNamespace(yaw_begin_m=.8, yaw_end_m=1.2)
        combined = segment(0, 3, 9.)
        planner = self.planner({})
        calls = []

        def make(kind, start, end, *args, **kwargs):
            if kind == 'WINDOW':
                calls.append(kwargs)
                return combined, []
            return None, []

        planner.make = make
        result = planner.shortcut([lead, window], ())
        self.assertEqual(result, [combined])
        self.assertEqual(calls[0]['corner'], window.corner)
        self.assertAlmostEqual(calls[0]['window_m'], .2)


if __name__ == '__main__':
    unittest.main()
