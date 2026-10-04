"""Road acceptance must not hide an intermediate rejected physical interval."""
from types import SimpleNamespace

from test_realistic_pedelec_acceptance import _interval_violations, _violations_after


def test_earlier_interval_failure_survives_clean_period_endpoint():
    samples = [SimpleNamespace(interval_id=0,time_s=.6,
                   channels={'attachment_violations':('rider_power.rider_hip_front','energy.constraint_work')}),
               SimpleNamespace(interval_id=1,time_s=.60125,
                   channels={'attachment_violations':()})]
    rows = [{'t':.61,'interval_violations':_interval_violations(samples)}]
    assert _violations_after(rows) == ((0,.6,('rider_power.rider_hip_front','energy.constraint_work')),)


def test_startup_grace_uses_each_violation_interval_time():
    rows = [{'t':.51,'interval_violations':((10,.5,('saddle.normal',)),)},
            {'t':.52,'interval_violations':((11,.50125,('rider_strength.rider_hip_rear',)),)}]
    assert _violations_after(rows) == ((11,.50125,('rider_strength.rider_hip_rear',)),)
