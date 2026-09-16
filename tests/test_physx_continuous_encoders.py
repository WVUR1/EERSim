"""Completed-tick continuous encoders with a mocked PhysX process; no scene."""
from types import SimpleNamespace

import numpy as np
import pytest

from eersim.physx_world import PhysXAdapter
from eersim.tuning_simulation import motor_sample


def state(q, qd=None):
    return {'q':np.asarray([q],dtype=np.float32),
            'qd':np.asarray([qd or [0.] * len(q)],dtype=np.float32),
            'poses':np.zeros((0,7),np.float32),'velocities':np.zeros((0,6),np.float32),
            'contacts':[]}


def fixture(initial=(6., 0., 0.)):
    joints=[{'body_id':7,'joint_index':index,'name':name,'path':'/robot/'+name,
             'type':kind,'articulation':True}
            for index,(name,kind) in enumerate([('wheel','continuous'),('arm','revolute'),('lift','prismatic')])]
    process=SimpleNamespace(initial={'actor_paths':[],'dof_names':['wheel','arm','lift'],
                                     'state':state(initial)})
    adapter=PhysXAdapter(SimpleNamespace(client=0,robot=7,dt=1/120),process,
                         {'actors':[],'joints':joints})
    return adapter,process


def step(adapter,process,q,qd=None):
    result=state(q,qd)
    before=result['q'].copy()
    process.call=lambda command,controls:result
    adapter.stepSimulation()
    np.testing.assert_array_equal(result['q'],before)
    return adapter.getJointState(7,0)[0]


@pytest.mark.parametrize('direction',[1,-1])
def test_completed_ticks_unwrap_multiple_turns_and_reverse_without_read_side_effects(direction):
    start=direction*6.
    adapter,process=fixture((start,2.,3.))
    assert adapter.getJointState(7,0)[0] == start
    positions=list(np.arange(start,start+direction*52,direction*.4))
    positions+=list(np.arange(positions[-1],start,-direction*.4))
    previous=positions[0]
    for continuous in positions:
        raw=(continuous+2*np.pi)%(4*np.pi)-2*np.pi
        observed=step(adapter,process,[raw,2.,3.],[(continuous-previous)*120,0,0])
        assert observed == pytest.approx(continuous,abs=5e-7)
        offsets=adapter._continuous_offsets.copy()
        for _ in range(8):
            assert adapter.getJointState(7,0)[0] == observed
            assert adapter.getJointStates(7,[0,1,2])[0][0] == observed
        assert adapter._continuous_offsets == offsets
        previous=continuous


def test_native_initial_value_and_unwrapped_finite_joints_are_preserved():
    adapter,process=fixture((6.1,40.,.2))
    assert adapter.getJointState(7,0)[0] == float(np.float32(6.1))
    value=step(adapter,process,[-6.1,-40.,12.],[-3.,4.,5.])
    assert value == float(np.float32(-6.1)) + 4*np.pi
    assert adapter.getJointState(7,0)[1] == -3.
    assert adapter.getJointState(7,1)[0] == -40.
    assert adapter.getJointState(7,2)[0] == 12.
    # A reset builds a new adapter instead of retaining old encoder windings.
    reset,_=fixture((-6.1,-40.,12.))
    assert reset.getJointState(7,0)[0] == float(np.float32(-6.1))
    assert reset._continuous_offsets == {0:0.}


def test_failed_native_step_does_not_advance_encoder_tracking():
    adapter,process=fixture()
    initial=adapter._state
    def fail(*args): raise RuntimeError('native step failed')
    process.call=fail
    with pytest.raises(RuntimeError,match='native step failed'):
        adapter.stepSimulation()
    assert adapter._state is initial
    assert adapter._continuous_raw == {0:6.}
    assert adapter._continuous_offsets == {0:0.}


def test_external_motor_ratio_and_reversal_apply_once_after_joint_unwrapping():
    adapter,process=fixture()
    joint=step(adapter,process,[-6.,0.,0.],[2.,0,0])
    # This is the existing World._observe mapping; the adapter stays radians.
    snapshot={'time':1.,'motors':{'1':{'position':joint*2*-1,'velocity':2.*2*-1}}}
    sample=motor_sample(snapshot,{'motors':{}},'capture')
    assert sample['motors']['1']['position_deg'] == pytest.approx(-(4*np.pi-6)*2*180/np.pi)
    assert sample['motors']['1']['velocity_rpm'] == pytest.approx(-4*30/np.pi)
