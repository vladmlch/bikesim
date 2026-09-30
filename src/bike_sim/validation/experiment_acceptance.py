"""Whole-experiment separation and explicit calibration gates.

A boolean gate describes evidence; it does not authenticate the provenance of
measurements. Release creation additionally verifies the referenced datasets.
"""


def validate_split(fit_experiment_ids: list[str], holdout_experiment_ids: list[str]) -> None:
    groups=(fit_experiment_ids,holdout_experiment_ids)
    if any(not isinstance(g,list) or not g for g in groups):
        raise ValueError('fit and holdout experiments are both required as lists')
    for group in groups:
        if any(not isinstance(v,str) or not v.strip() or v!=v.strip() for v in group):
            raise ValueError('experiment identifiers must be nonempty canonical strings')
        if len(set(group))!=len(group):
            raise ValueError('duplicate experiment identifier')
    if set(groups[0]) & set(groups[1]):
        raise ValueError('fit/holdout leakage at experiment level')


def is_synthetic(payload: dict) -> bool:
    flag=payload.get('synthetic',False)
    if type(flag) is not bool:
        raise ValueError('synthetic declaration must be a bool')
    source=str(payload.get('source','')).strip().lower()
    return flag or source.startswith(('synthetic','simulation:','fixture:'))


def accepted_calibration_status(*, synthetic: bool, holdout_passed: bool,
                                converged: bool, within_scope: bool) -> str:
    if not all(type(v) is bool for v in (synthetic,holdout_passed,converged,within_scope)):
        raise ValueError('calibration gates require boolean evidence')
    if synthetic or not (holdout_passed and converged and within_scope):
        return 'parameterized_unvalidated'
    return 'validated_within_declared_scope'


MODEL_DISCREPANCY_BUDGET={
    'budget_id':'bikesim-initial-holdout-2026-09-30',
    'axle_load_weight_fraction':.03,
    'suspension_travel_rmse_m':.003,
    'pitch_rmse_rad':0.017453292519943295,
    'pitch_rate_rmse_rad_s':.05,
}
