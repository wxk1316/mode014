"""Static cluster occlusion and physically reachable association regressions."""
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'rda_dwa_real'),str(ROOT/'rda_dwa')]
from rda_dwa_real.kf_tracker import GlobalKFTracker


def detection(x,y=0.):return (x,y,'pedestrian',.15,.15)


class StationarityTests(unittest.TestCase):
    def test_confirmed_static_clusters_remain_static_on_alternating_misses(self):
        tracker=GlobalKFTracker(dt=.1,static_hold_frames=10,lost_ttl=10)
        for _ in range(12):tracker.step([detection(0.),detection(.8)])
        for i in range(20):
            outputs=tracker.step([detection(0. if i%2==0 else .8)])
            self.assertEqual(len(outputs),2)
            self.assertTrue(all(o[5]=='block' and o[3]==0. for o in outputs),outputs)

    def test_alternating_static_points_are_not_joined_as_one_moving_track(self):
        tracker=GlobalKFTracker(dt=.1,gate=1.,static_hold_frames=10,lost_ttl=10)
        for i in range(40):outputs=tracker.step([detection(0. if i%2==0 else .8)])
        self.assertEqual(len(outputs),2)
        self.assertEqual(sorted(round(o[1],3) for o in outputs),[0.,.8])
        self.assertTrue(all(o[3]==0. and o[5]=='block' for o in outputs),outputs)

    def test_predictions_do_not_confirm_an_unobserved_new_track(self):
        tracker=GlobalKFTracker(dt=.1,static_hold_frames=5,lost_ttl=10)
        tracker.step([detection(0.)])
        for _ in range(7):outputs=tracker.step([detection(3.)])
        hidden=next(o for o in outputs if o[0]==1)
        self.assertEqual(hidden[5],'unknown')
        self.assertEqual(tracker.tracks[1]._st_cnt,1)

    def test_confirmed_static_track_still_expires_at_original_miss_ttl(self):
        tracker=GlobalKFTracker(dt=.1,static_hold_frames=3,lost_ttl=4)
        for _ in range(5):tracker.step([detection(0.)])
        for _ in range(3):self.assertEqual(tracker.step([])[0][5],'block')
        self.assertEqual(tracker.step([]),[])

    def test_real_motion_is_tracked_and_missed_mover_keeps_dynamic_class(self):
        tracker=GlobalKFTracker(dt=.1,gate=1.,static_hold_frames=10,lost_ttl=10)
        for i in range(40):outputs=tracker.step([detection(i*.1)])
        self.assertEqual(len(outputs),1)
        self.assertEqual(outputs[0][0],1)
        self.assertEqual(outputs[0][5],'pedestrian')
        self.assertGreater(outputs[0][3],.7)
        self.assertEqual(tracker.step([])[0][5],'pedestrian')

    def test_reachability_uses_elapsed_source_time_across_missed_frames(self):
        tracker=GlobalKFTracker(dt=.1,gate=1.,static_hold_frames=10,lost_ttl=10)
        tracker.step([detection(0.)])
        for _ in range(3):tracker.step([])
        output=tracker.step([detection(.5)],dt=.1)
        self.assertEqual(len(output),1)
        self.assertEqual(output[0][0],1)


if __name__=='__main__':unittest.main()
