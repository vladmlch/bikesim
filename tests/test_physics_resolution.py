from dataclasses import asdict, FrozenInstanceError
import pytest
from bike_sim.physics.resolution import resolve_physics_config,load_physics_config


def test_legacy_default_unchanged_and_physical_default_coasts():
    a=resolve_physics_config();b=resolve_physics_config({'physics_mode':'physical'})
    assert a.physics_mode=='legacy' and a.drive_mode=='ideal_speed_control' and a.pitch_assist
    assert b.drive_mode=='coast' and not b.pitch_assist and b.initial_speed_mps==0.


def test_nested_toml_and_explicit_cli_zero_have_correct_precedence(tmp_path):
    file=tmp_path/'p.toml'
    file.write_text('physics_mode="physical"\ninitial_speed_mps=2.0\n[drive]\nhuman_torque_nm=25\n[drive.assist]\ngain=3.0\nmax_power=250.0\n')
    cfg=load_physics_config(file,{'initial_speed_mps':0.,'drive':{'assist':{'gain':0.}}})
    assert cfg.drive.human_torque_nm==25. and cfg.drive.assist.gain==0.
    assert cfg.drive.assist.max_power==250. and cfg.initial_speed_mps==0.
    with pytest.raises(FrozenInstanceError): cfg.drive.assist.gain=2.


@pytest.mark.parametrize('values',[
 {'physics_mode':'physica'}, {'timestep_s':True}, {'initial_speed_mps':float('nan')},
 {'tires':{'backend':'fallback'}}, {'drive':{'unknown':3}}, {'unknown':3},
 {'physics_mode':'physical','pitch_assist':True}, {'pitch_assist':1},
 {'drive':{'gearing':{'front_teeth':3.5}}}, {'tires':{'front':{'mu':-1}}},
 {'resistance':{'wind_world_mps':[1,1,0]}},
])
def test_conflicting_or_unsupported_values_rejected(values):
    with pytest.raises(ValueError): resolve_physics_config(values)


def test_round_trip_full_config_and_no_input_mutation():
    values={'physics_mode':'physical','tires':{'front':{'material':{
      'radial_k_n_m':120000.,'radial_c_ns_m':500.,'pressure_pa_gauge':170000.,
      'provenance':'synthetic','valid_load_range_n':[0,1500]}}}}
    cfg=resolve_physics_config(values)
    assert asdict(resolve_physics_config(asdict(cfg)))==asdict(cfg)
    assert isinstance(values['tires']['front']['material']['valid_load_range_n'],list)
