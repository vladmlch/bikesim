from bike_sim.physics.checks import scalar


def rider_work_ledger(component_work_j: dict) -> dict[str, float]:
    work = {name: scalar(value, name) for name, value in component_work_j.items()}
    active = sum((value for name, value in work.items() if name.startswith('act_rider_')), 0.0)
    passive = work.get('rider_passive_damping', 0.)
    return {'active_joint_work_j': active, 'passive_joint_work_j': passive,
            'net_joint_work_j': active + passive,
            'motor_mechanical_work_j': work.get('mid_drive', 0.)}
