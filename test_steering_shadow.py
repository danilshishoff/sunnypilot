import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from steering_shadow import ShadowPair, PastSamples


class FakeController:
  def __init__(self):
    self.pid = SimpleNamespace(i=0.)
    self.steer_max = 1.

  def update_limits(self):
    self.limit = self.steer_max

  def reset(self):
    self.pid.i = 0.

  def update_live_torque_params(self, *args):
    pass

  def update(self, active, demand):
    self.pid.i += .01 if active else 0
    value = max(-self.limit, min(self.limit, demand)) if active else 0.
    return value, 0., SimpleNamespace(saturated=abs(value) >= self.limit)


class ShadowTests(unittest.TestCase):
  def test_buffer_never_selects_future(self):
    history = PastSamples(['carState'])
    history.add('carState', 10, 'old', True)
    history.add('carState', 20, 'future', True)
    self.assertIsNone(history.at('carState', 9))
    self.assertEqual(history.at('carState', 15)[1], 'old')
    self.assertEqual(history.at('carState', 20)[1], 'future')

  def test_warmup_and_reset(self):
    pair = ShadowPair(FakeController)
    for _ in range(200):
      row = pair.update(True, (.5,), .5)
    self.assertFalse(row['warmup'])
    pair.reset()
    self.assertTrue(pair.update(True, (.5,), .5)['warmup'])

  def test_nonfinite_output_rejected(self):
    class Broken(FakeController):
      def update(self, *args):
        return float('nan'), 0., SimpleNamespace(saturated=False)
    with self.assertRaises(ValueError):
      ShadowPair(Broken).update(True, (1.,), 0.)

  def test_independent_state(self):
    pair = ShadowPair(FakeController)
    pair.shadow.pid.i = 10
    self.assertEqual(pair.baseline.pid.i, 0)

  def test_shared_state_rejected(self):
    shared = FakeController()
    with self.assertRaises(ValueError):
      ShadowPair(lambda: shared)

  def test_separate_caps_and_sign(self):
    pair = ShadowPair(FakeController)
    for sign in (-1, 1):
      row = pair.update(True, (sign * 2.,), sign)
      self.assertAlmostEqual(row['baseline_units'], sign * 300)
      self.assertAlmostEqual(row['shadow_units'], sign * 500)

  def test_inactive_zero(self):
    pair = ShadowPair(FakeController)
    pair.update(True, (2.,), 1.)
    row = pair.update(False, (2.,), 0.)
    self.assertEqual(row['shadow_units'], 0)
    self.assertTrue(row['warmup'])

  def test_no_output_transport(self):
    tree = ast.parse(Path(__file__).with_name('steering_shadow.py').read_text())
    blocked = {'PubMaster', 'pub_sock', 'send', 'sendcan', 'CarController', 'Controls', 'Panda'}
    names = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    names |= {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    self.assertFalse(names & blocked)


if __name__ == '__main__':
  unittest.main()
