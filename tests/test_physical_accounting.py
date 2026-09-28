from types import SimpleNamespace
import numpy as np
import pytest
from bike_sim.sim.ride.physical_energy import gas_chamber_energy, fork_air_energy
from bike_sim.sim.ride.physical_samples import PhysicalSample, WorkHistory


@pytest.mark.parametrize('gamma', [1., 1.00001, 1.2, 1.4])
def test_gas_work_matches_pressure_integral(gamma):
    p, v0, v = 800000., .00025, .00014
    volumes = np.linspace(v, v0, 100001)
    integral = np.trapezoid(p*(v0/volumes)**gamma, volumes)
    assert gas_chamber_energy(p,v0,v,gamma) == pytest.approx(integral, rel=1e-10)


def test_fork_energy_integrates_both_chambers_and_terminal_force():
    s = SimpleNamespace(total_travel_mm=180.,base_pos_volume_m3=.00035,
                        base_neg_volume_m3=.00008,token_volume_m3=.00001,
                        piston_area_m2=.001,gamma=1.3)
    p = 800000.
    def force(x_mm):
        x=np.clip(x_mm/1000.,0.,.18)
        return .001*p*((.00033/(.00033-.001*x))**1.3-(.00008/(.00008+.001*x))**1.3)
    spring = SimpleNamespace(specs=s,num_tokens=2,abs_pressure_pa=p,compute_axial_force=force)
    x=np.linspace(0,.19,100001)
    assert fork_air_energy(spring,.19) == pytest.approx(np.trapezoid(force(x*1000),x),rel=1e-9)
    assert fork_air_energy(spring,-.01)==0.


def sample(i, value=3.):
    patches=({'source_geom':'terrain','normal_load_n':value},)
    return PhysicalSample(i,i*.01,(i+1)*.01,np.array([0.]),np.array([2.]),
                          {'motor':np.array([3.])},{'tires':{'front':{'patches':patches},'rear':{'patches':()}}})


def test_nested_sample_and_arrays_are_detached_and_immutable():
    q=np.array([2.]); v=np.array([3.]); f=np.array([4.]); nested={'x':[1.,2.]}
    s=PhysicalSample(0,0.,.1,q,v,{'motor':f},nested)
    q[0]=v[0]=f[0]=10.; nested['x'][0]=42.
    assert s.powers_w['motor']==12.
    assert s.channels['x']==(1.,2.)
    with pytest.raises(ValueError): s.qvel.setflags(write=True)
    with pytest.raises(TypeError): s.channels['x']=5


def test_substep_work_and_raw_airtime_do_not_depend_on_storage_stride():
    totals=[]
    for stride in (1,10):
        history=WorkHistory(); rows=[]
        for i in range(100):
            s=sample(i)
            history.add(s)
            if i%stride==0: rows.append(s)
        totals.append((history.work_j,history.airtime_s))
    assert totals[0]==totals[1]
    assert totals[0][0]['motor']==pytest.approx(6.)
    assert totals[0][1]['front']['0.0']==0.
    assert totals[0][1]['front']['1.0']==0.
    assert totals[0][1]['front']['5.0']==pytest.approx(1.)
    assert totals[0][1]['rear']['0.0']==pytest.approx(1.)


def test_duplicate_and_missing_intervals_rejected():
    h=WorkHistory(); h.add(sample(0))
    with pytest.raises(ValueError): h.add(sample(0))
    with pytest.raises(ValueError): h.add(sample(2))
    h.add(sample(1))


def test_missing_raw_contact_evidence_is_not_counted_as_airtime():
    h = WorkHistory()
    s = PhysicalSample(0, 0., .01, np.array([0.]), np.array([2.]),
                       {'motor': np.array([3.])}, {})
    with pytest.raises(ValueError, match='raw contact'):
        h.add(s)
    assert h.duration_s == 0.
    assert h.work_j == {}
    assert h.last_id is None


def test_unknown_contact_source_does_not_partially_update_history():
    h = WorkHistory()
    s = PhysicalSample(0, 0., .01, np.array([0.]), np.array([2.]),
        {'motor': np.array([3.])}, {'tires': {'front': {'patches': ()},
        'rear': {'patches': ({'normal_load_n': 5., 'source_geom': 'unknown'},)}}})
    with pytest.raises(ValueError, match='contact source'):
        h.add(s)
    assert h.duration_s == 0.
    assert h.work_j == {}
