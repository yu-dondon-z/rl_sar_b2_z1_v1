import json
from pathlib import Path
import tempfile
import unittest

from camera_cpu import PublicationTiming
from camera_diagnostics import DiagnosticWriter


class TimingTests(unittest.TestCase):
    def test_measures_previous_frame_expiry_without_refreshing_it(self):
        clock=PublicationTiming()
        a=dict(valid=True,capture_monotonic_ns=1_000_000_000,publish_monotonic_ns=1_200_000_000)
        clock.annotate(a)
        self.assertIsNone(a['previous_valid_age_at_publication_ms'])
        bad=dict(valid=False,capture_monotonic_ns=1_240_000_000,publish_monotonic_ns=1_510_000_000)
        clock.annotate(bad)
        self.assertEqual(bad['previous_valid_age_at_publication_ms'],510.)
        self.assertEqual(clock.last_valid_capture_ns,1_000_000_000)
        good=dict(valid=True,capture_monotonic_ns=1_520_000_000,publish_monotonic_ns=1_660_000_000)
        clock.annotate(good)
        self.assertEqual(good['previous_valid_age_at_publication_ms'],660.)
        self.assertEqual(clock.last_valid_capture_ns,1_520_000_000)
        self.assertEqual(good['capture_monotonic_ns'],1_520_000_000)

    def test_packet_journal_is_post_publication_diagnostic_copy(self):
        with tempfile.TemporaryDirectory() as d:
            writer=DiagnosticWriter(d)
            p=dict(sequence=1,valid=True,capture_monotonic_ns=100,publish_monotonic_ns=200)
            self.assertTrue(writer.packet(p));p['capture_monotonic_ns']=999
            self.assertTrue(writer.close(2.))
            got=json.loads((Path(d)/'guide_packets.jsonl').read_text())
            self.assertEqual(got['capture_monotonic_ns'],100)
            self.assertEqual(writer.errors,0)


if __name__=='__main__':unittest.main()
