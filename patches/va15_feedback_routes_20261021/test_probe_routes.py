import ast
import math
from pathlib import Path
import unittest
from probe_routes import clean, finite_tree, summarize

def row(source,received=1.,data=None):
    data={'body_height':.52} if data is None else data
    return dict(source=source,received=received,data=data,values_finite=finite_tree(data))

class Boundaries(unittest.TestCase):
    def test_no_frames(self):
        r=summarize([],2.)
        self.assertFalse(r['source_time_advancing_at_end'])
        self.assertIsNone(r['last'])
    def test_zero_stamp_arrivals_not_source_time(self):
        r=summarize([row(0,1.),row(0,1.9)],2.)
        self.assertFalse(r['source_time_advancing_at_end'])
    def test_constant_positive_stamp_is_stale(self):
        r=summarize([row(10,1.),row(10,1.99)],2.)
        self.assertFalse(r['source_time_advancing_at_end'])
        self.assertGreater(r['last_source_advance_age_s'],.15)
    def test_time_advance_not_physical_permission(self):
        r=summarize([row(10,1.),row(20,1.99)],2.)
        self.assertTrue(r['source_time_advancing_at_end'])
        self.assertFalse(r['control_authorized'])
        self.assertFalse(r['physical_frame_height_velocity_validated'])
    def test_source_regression_not_accepted(self):
        self.assertFalse(summarize([row(20,1.),row(10,1.99)],2.)['source_time_advancing_at_end'])
    def test_nonfinite_not_zero_or_valid(self):
        self.assertEqual(clean({'v':[math.nan,math.inf]}),{'v':[None,None]})
        self.assertFalse(summarize([row(10,1.),row(20,1.99,{'h':math.nan})],2.)['source_time_advancing_at_end'])
    def test_no_control_imports_or_calls(self):
        tree=ast.parse(Path(__file__).with_name('probe_routes.py').read_text())
        names={alias.name for n in ast.walk(tree) if isinstance(n,ast.ImportFrom) and n.module and
               n.module.startswith('unitree_sdk2py') for alias in n.names}
        self.assertEqual(names,{'ChannelFactoryInitialize','ChannelSubscriber','LowState_','SportModeState_','Odometry_'})
        forbidden={'ChannelPublisher','ServiceSwitch','ReleaseMode','SetCommand','setFsm'}
        self.assertFalse(any(isinstance(n,ast.Attribute) and n.attr in forbidden for n in ast.walk(tree)))

if __name__=='__main__': unittest.main()
