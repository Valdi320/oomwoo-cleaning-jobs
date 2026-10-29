"""Adversarial synthetic timing and Nav2 handoff transitions; no ROS2/hardware."""
import unittest
from oomwoo_cleaning_jobs.reactive_velocity import MotionHealth, ReactiveMuxError, ThreeWayVelocityMux
from oomwoo_cleaning_jobs.dock_handoff import Velocity

def health(*, active=None):
    return MotionHealth(pose_fresh=True, resources_ok=True, map_current=True,
                        safety_loop_ok=True, isolated_sources=True,
                        nav2_idle=active is None, nav2_active_goal=active,
                        dock_inactive=True, sweep_inactive=True)

class ContinuousOdomAndNavBoundary(unittest.TestCase):
    def test_quiet_requires_continuous_odom_not_two_distant_samples(self):
        m=ThreeWayVelocityMux()
        m.observe_odom(now=1.0,linear_mps=0,angular_rps=0)
        m.observe_odom(now=1.85,linear_mps=0,angular_rps=0)
        m.observe_health(now=1.86,proof=health())
        with self.assertRaises(ReactiveMuxError):
            m.claim_sweep(now=1.90,nav_released=True,prior_terminal_uuid=None)
        self.assertEqual(m.owner,m.STOP)

    def test_quiet_interval_invalidated_by_midstream_sensor_gap(self):
        m=ThreeWayVelocityMux()
        for now in [1.,1.12,1.24,1.36,2.,2.12,2.24,2.36,2.48,2.60,2.72]:
            m.observe_odom(now=now,linear_mps=0,angular_rps=0)
        m.observe_health(now=2.73,proof=health())
        with self.assertRaises(ReactiveMuxError):
            m.claim_sweep(now=2.74,nav_released=True,prior_terminal_uuid=None)
        self.assertEqual(m.owner,m.STOP)

    def test_nav_active_idle_active_cannot_resurrect_prior_twist(self):
        m=ThreeWayVelocityMux()
        for now in [1.,1.12,1.24,1.36,1.48,1.60,1.72,1.84]:
            m.observe_odom(now=now,linear_mps=0,angular_rps=0)
        m.observe_health(now=1.85,proof=health())
        uuid="a"*32
        token=m.claim_transit(now=1.90,nav_goal_uuid=uuid)
        m.observe_health(now=1.92,proof=health(active=uuid))
        m.observe_odom(now=1.93,linear_mps=0,angular_rps=0)
        m.accept_twist(source=m.NAV,token=token,now=1.94,velocity=Velocity(.1,0))
        self.assertEqual(m.output(now=1.95),Velocity(.1,0))
        m.observe_health(now=1.96,proof=health())
        self.assertEqual(m.output(now=1.97),Velocity())
        with self.assertRaises(ReactiveMuxError):
            m.observe_health(now=1.98,proof=health(active=uuid))
        self.assertEqual(m.owner,m.STOP)
        self.assertEqual(m.output(now=1.99),Velocity())

    def test_healthy_continuous_quiet_and_terminal_release_still_allowed(self):
        m=ThreeWayVelocityMux()
        for now in [1.,1.12,1.24,1.36,1.48,1.60,1.72,1.84]:
            m.observe_odom(now=now,linear_mps=0,angular_rps=0)
        m.observe_health(now=1.85,proof=health())
        uuid="b"*32
        m.claim_transit(now=1.90,nav_goal_uuid=uuid)
        m.observe_health(now=1.92,proof=health(active=uuid))
        m.observe_health(now=1.93,proof=health())
        m.release_transit(now=1.94,terminal_uuid=uuid,terminal_verified=True,
                          nav_input_isolated=True)
        for now in [2.,2.12,2.24,2.36,2.48,2.60,2.72,2.84]:
            m.observe_odom(now=now,linear_mps=0,angular_rps=0)
        m.observe_health(now=2.85,proof=health())
        token=m.claim_sweep(now=2.90,nav_released=True,prior_terminal_uuid=uuid)
        self.assertEqual(m.owner,m.SWEEP)
        self.assertEqual(len(token),32)

if __name__=="__main__":
    unittest.main()
