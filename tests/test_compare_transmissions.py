from tools.compare_transmissions import compare_metrics


def _metrics(onset=1.0, fraction=.30, min_load=200., stroke=.020, speed=3.0):
    episodes = [] if onset is None else [dict(start_s=onset, confirmed=True)]
    return dict(mean_speed_mps=speed, max_shock_stroke_m=stroke,
                wheelie=dict(front_load_fraction_min=fraction, min_front_load_n=min_load,
                             wheelie_episode_records=episodes))


def test_identical_runs_pass():
    result = compare_metrics(_metrics(), _metrics())
    assert result['passed'] and all(c['passed'] for c in result['checks'].values())


def test_onset_shift_beyond_50ms_fails():
    assert compare_metrics(_metrics(), _metrics(onset=1.04))['passed']
    result = compare_metrics(_metrics(), _metrics(onset=1.06))
    assert not result['passed'] and not result['checks']['onset_s']['passed']


def test_wheelie_present_in_only_one_run_fails():
    assert not compare_metrics(_metrics(), _metrics(onset=None))['passed']
    assert compare_metrics(_metrics(onset=None), _metrics(onset=None))['passed']


def test_relative_tolerances():
    assert not compare_metrics(_metrics(), _metrics(min_load=160.))['passed']   # 20 % > 15 %
    assert compare_metrics(_metrics(), _metrics(min_load=180.))['passed']       # 10 %
    assert not compare_metrics(_metrics(), _metrics(stroke=.0225))['passed']    # 12.5 % > 10 %
    assert not compare_metrics(_metrics(), _metrics(speed=3.2))['passed']       # 6.7 % > 5 %


def test_only_confirmed_episodes_define_onset():
    ref = _metrics(onset=None)
    ref['wheelie']['wheelie_episode_records'] = [dict(start_s=.2, confirmed=False)]
    assert compare_metrics(ref, _metrics(onset=None))['passed']
