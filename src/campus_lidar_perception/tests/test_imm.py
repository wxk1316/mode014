import numpy as np
import pytest
from campus_lidar_perception.filters import IMMKalman
from campus_lidar_perception.tracker import MultiObjectTracker,TrackerConfig
from campus_lidar_perception.types import Detection

def test_imm_total_covariance_includes_between_models():
    imm=IMMKalman(np.zeros(4),np.eye(4),cv_probability=.5)
    imm.means[1,0]=10.;imm._fuse()
    assert imm.state[0]==pytest.approx(5.)
    assert imm.covariance[0,0]==pytest.approx(26.)
    imm._mix();imm._fuse()
    assert imm.state[0]==pytest.approx(5.) and imm.covariance[0,0]==pytest.approx(26.)

def test_imm_probabilities_and_numerical_stability():
    imm=IMMKalman(np.zeros(4),np.eye(4));time=0.
    for i in range(300):
        dt=[.07,.1,.15][i%3];time+=dt;imm.predict(dt)
        imm.update([3*np.sin(time*.4),2*np.cos(time*.3)],.1);imm.limit_speed(15.)
        assert np.isfinite(imm.state).all() and np.isfinite(imm.covariance).all()
        assert sum(imm.probabilities)==pytest.approx(1.) and min(imm.probabilities)>=0.
        assert np.linalg.eigvalsh(imm.covariance).min()>-1e-9
    assert not np.allclose(imm.probabilities,[.7,.3])

def test_imm_moving_activation_and_static_cv_restore():
    tracker=MultiObjectTracker(TrackerConfig(tracking_filter='imm'))
    for i in range(50):
        row=tracker.step([Detection(.06*i,2.,label='PEDESTRIAN')],1.+i*.1)[0]
        assert row['configured_filter']=='imm'
        if i==0:assert row['active_filter']=='cv' and not row['moving']
    assert row['active_filter']=='imm' and row['speed']==pytest.approx(.6,abs=.06)
    assert row['filter_model_names']==['CV','CA']
    for i in range(120):row=tracker.step([Detection(2.94,2.,label='PEDESTRIAN')],6.+i*.1)[0]
    assert row['active_filter']=='cv' and row['speed']==0. and not row['moving']

def test_filter_switch_preserves_state_and_covariance():
    tracker=MultiObjectTracker(TrackerConfig(tracking_filter='imm'));tracker.step([Detection(0.,2.)],1.)
    track=tracker.tracks[1];state=track.state.copy();p=track.covariance.copy()
    track.moving=True;track._select_filter()
    np.testing.assert_allclose(track.state,state);np.testing.assert_allclose(track.covariance,p)
    track.moving=False;track._select_filter()
    np.testing.assert_allclose(track.state,state);np.testing.assert_allclose(track.covariance,p)

def test_expired_id_does_not_reassociate_after_gap():
    tracker=MultiObjectTracker();one=tracker.step([Detection(1.,2.)],1.)[0]
    two=tracker.step([Detection(1.,2.)],1.6)[0];assert one['id']!=two['id']

def test_imm_crossing_and_missing_observations():
    tracker=MultiObjectTracker(TrackerConfig(tracking_filter='imm'))
    for i in range(60):
        a=Detection(-2.+.08*i,0.,label='PEDESTRIAN');b=Detection(2.-.08*i,0.,label='BICYCLE',length=1.5)
        measurements=([a,b] if i%2==0 else [b,a]) if i not in (35,36) else [b]
        rows=tracker.step(measurements,1.+i*.1)
        assert [r['id'] for r in rows]==[1,2] and rows[0]['semantic_label']=='PEDESTRIAN'

def test_invalid_filter_config_rejected():
    with pytest.raises(ValueError):TrackerConfig(tracking_filter='mixed')
    with pytest.raises(ValueError):TrackerConfig(imm_stay_probability=1.)
    with pytest.raises(ValueError):TrackerConfig(position_sigma=float('nan'))

def test_imm_prediction_remains_within_speed_and_displacement_bounds():
    tracker=MultiObjectTracker(TrackerConfig(tracking_filter='imm'))
    for i in range(31):
        t=i*.1;row=tracker.step([Detection(.5*5*t*t,0.,label='CAR')],1.+t)[0]
    previous=np.array([row['x'],row['y']])
    predicted=tracker.step([],4.2)[0]
    assert predicted['predicted'] and predicted['speed']<=15.+1e-10
    assert np.linalg.norm(np.array([predicted['x'],predicted['y']])-previous)<=15*.2+1e-9
