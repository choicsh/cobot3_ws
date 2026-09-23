import math
import numpy as np
from nav_to_goal.hospital_docking import (
    fit_table_edge, table_front_envelope, docking_command, TABLES,
)
from nav_to_goal.hospital_safety import observation_age


def test_table_fit_rejects_corner_and_outliers():
    x = np.linspace(-1.2, .7, 50)
    y = .008*x-.55 + .002*np.sin(7*x)
    points = list(zip(x,y))+[(.5, -.3),(.7,-.8),(-1.,-.9)]
    slope, offset, count = fit_table_edge(points)
    assert abs(slope-.008)<.003
    assert abs(offset+.55)<.003
    assert count>=45


def test_table_fit_requires_real_span_and_sufficient_returns():
    assert fit_table_edge([(0.,-.55)]*20) is None
    assert fit_table_edge([(0.,-.55),(1.,-.55)]) is None


def test_dense_tabletop_returns_do_not_replace_front_edge():
    x = np.linspace(-1.2, .7, 200)
    edge = np.column_stack((x, .005*x-.55))
    top = np.concatenate([np.column_stack((x, .005*x-.55-depth-.03*x*x))
                          for depth in np.linspace(.02, .3, 30)])
    samples = np.concatenate((edge, top, [[0., -.32]]))
    slope, offset, count = fit_table_edge(table_front_envelope(samples))
    assert abs(slope-.005) < .001
    assert abs(offset+.55) < .001
    assert count > 50
    assert table_front_envelope([]).shape == (0, 2)


def test_forward_reverse_docking_steer_toward_table():
    vf,wf=docking_command(-1.,0.,.05,1.)
    vr,wr=docking_command(1.,0.,.05,-1.)
    assert vf>0 and wf<0 and vr<0 and wr>0
    assert abs(vf)<=.18 and abs(vr)<=.18
    assert docking_command(.005,0.,0.,1.)==(0.,0.)


def test_clock_skew_does_not_accept_stale_or_far_future_data():
    assert observation_age(10.,10.033)==0.
    assert observation_age(10.,10.05) is None
    assert observation_age(10.,9.39) is None
    assert math.isclose(observation_age(10.,9.5),.5)


def test_dock_longitudinal_chassis_center_matches_authored_table():
    for table in TABLES.values():
        center=(table['x_min']+table['x_max'])/2
        assert abs(table['dock_x']+.45-center)<1e-6


def test_mapped_wall_corner_is_not_a_moving_obstacle_but_open_space_is():
    from types import SimpleNamespace as NS
    from nav_to_goal.static_scan_mask import StaticScanMask
    values=np.zeros((50,50),dtype=int)
    values[:,20]=100
    values[25,35]=-1
    msg=NS(info=NS(width=50,height=50,resolution=.05,
        origin=NS(position=NS(x=-1.,y=-1.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=values.ravel())
    mask=StaticScanMask(msg)
    assert mask.contains(.025, .25)
    assert mask.contains(.125, .25)
    assert not mask.contains(.4, .25)
    assert not mask.contains(.775, .275)  # Unknown is not known wall.
    assert not mask.contains(20.,20.)


def test_static_scan_mask_respects_rotated_map_origin():
    from types import SimpleNamespace as NS
    from nav_to_goal.static_scan_mask import StaticScanMask
    values=np.zeros((4,4),dtype=int);values[0,0]=100
    msg=NS(info=NS(width=4,height=4,resolution=1.,
        origin=NS(position=NS(x=10.,y=20.),orientation=NS(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5)))),data=values.ravel())
    mask=StaticScanMask(msg,margin=0.)
    assert mask.contains(9.5,20.5)
    assert not mask.contains(10.5,20.5)
