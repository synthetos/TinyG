"""Adversarial N111 protocol tests, not firmware or trajectory validation."""
import itertools
import random
import unittest
from override_queue_model import QueueModel, Row


class QueueHandoff(unittest.TestCase):
    def test_all_short_schedules_preserve_old_plan_or_claim_whole_suffix(self):
        # 4096 interleavings, with execution/refill permitted between every
        # producer unit. Always-full four-slot ring wraps repeatedly.
        for schedule in itertools.product((0, 1), repeat=12):
            m = QueueModel(capacity=4, lead=2, row_cost=1, ramp=2)
            m.set_factor(.5)
            for event in schedule:
                m.producer() if event else m.execute()
                self.assertEqual(len(m.queue), 4)
            m.check()

    def test_missed_deadline_keeps_old_plan_then_retries(self):
        m = QueueModel()
        m.set_factor(.5)
        m.producer()
        for _ in range(20):
            m.execute()  # deliberately starve the producer past its anchor
        self.assertEqual(m.claims, 0)
        self.assertEqual(m.misses, 1)
        self.assertTrue(all(r.generation == 0 and r.exit == 1 for r in m.trace))
        for _ in range(100):
            for _ in range(6):
                m.producer()
            m.execute()
            self.assertEqual(len(m.queue), 32)
        self.assertEqual(m.claims, 1)
        self.assertGreaterEqual(m.retries, 2)
        self.assertEqual(m.trace[-1].exit, .5)
        m.check()

    def test_continuously_full_dense_queue_with_many_requests(self):
        for seed in range(40):
            rng = random.Random(seed)
            m = QueueModel()
            for tick in range(1000):
                if tick % 50 == 0:
                    m.set_factor((.5, 1.5, 1.)[(tick//50) % 3])
                for _ in range(rng.randint(5, 8)):
                    m.producer()
                m.execute()
                self.assertEqual(len(m.queue), 32)
            self.assertEqual(m.claims, 20, seed)
            m.check()

    def test_cancel_and_new_request_at_each_prepublication_point(self):
        for units in range(40):
            for cancel in (False, True):
                m = QueueModel(capacity=8, lead=4, row_cost=2, ramp=4)
                m.set_factor(.5)
                for _ in range(units):
                    m.producer()
                old_request = m.request
                m.cancel() if cancel else m.set_factor(1.5)
                for _ in range(40):
                    for _ in range(8):
                        m.producer()
                    m.execute()
                self.assertFalse(any(r.generation == old_request for r in m.trace))
                self.assertEqual(m.claims, 0 if cancel else 1)
                m.check()

    def test_claimed_bank_survives_new_request_until_promotion(self):
        m = QueueModel(capacity=8, lead=4, row_cost=1, ramp=4)
        m.set_factor(.5)
        for _ in range(5):
            for _ in range(8):
                m.producer()
            m.execute()
        self.assertEqual(m.claims, 1)
        m.set_factor(1.5)
        for _ in range(40):
            for _ in range(8):
                m.producer()
            m.execute()
        self.assertEqual(m.claims, 2)
        m.check()

    def test_insufficient_service_is_safe_but_not_live(self):
        # No protocol can guarantee a new plan when CPU service is insufficient.
        # Report this regime instead of calling repeated retries success.
        m = QueueModel(row_cost=4)
        m.set_factor(.5)
        for _ in range(500):
            m.producer()
            m.execute()
        self.assertEqual(m.claims, 0)
        self.assertGreater(m.misses, 1)
        self.assertTrue(m.pending)
        self.assertTrue(all(r.generation == 0 for r in m.trace))
        m.check()

    def test_recycled_slot_or_partial_suffix_cannot_be_claimed(self):
        for damaged in ("recycled", "partial"):
            m = QueueModel(capacity=8, lead=4, row_cost=1, ramp=4)
            m.set_factor(.5)
            for _ in range(4):
                for _ in range(8):
                    m.producer()
                m.execute()
            for _ in range(8):
                m.producer()
            c = m.candidate
            self.assertTrue(c.ready)
            if damaged == "recycled":
                row = c.rows[c.anchor]
                # Same physical ring slot, preceding incarnation.
                c.rows[c.anchor] = Row(row.block-8,row.generation,row.entry,row.exit)
            else:
                del c.rows[max(c.rows)]
            m.execute()
            self.assertEqual(m.claims,0)
            self.assertEqual(m.misses,1)
            self.assertEqual(m.trace[-1].generation,0)
            m.check()

    def test_retry_moves_anchor_farther_ahead_under_marginal_service(self):
        m = QueueModel()
        m.set_factor(.5)
        for _ in range(150):
            for _ in range(3):
                m.producer()
            m.execute()
        self.assertGreater(m.misses,0)
        self.assertEqual(m.claims,1)
        self.assertGreater(m.lead,16)
        self.assertEqual(m.trace[-1].exit,.5)
        m.check()
