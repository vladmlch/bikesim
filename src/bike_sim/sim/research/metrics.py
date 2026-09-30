"""Compact per-episode outcome record for policy development.

This is an evaluation surface, not an observation: it reads simulator truth
(loads, progress) and must never be fed back to a policy.
"""


def episode_metrics(env):
    m = env.tracker.metrics
    duration = env.sim.time_s
    progress = float(env.sim.position_m)
    requested = env.torque_requested_nms
    return {
        'outcome': env.reason, 'duration_s': duration, 'progress_m': progress,
        'mean_speed_mps': progress/duration if duration > 0. else 0.,
        'finish_time_s': duration if env.reason == 'finish' else None,
        'torque_delivered_nms': env.torque_delivered_nms,
        'torque_requested_nms': requested,
        # Delivered/requested only exists when the policy commanded a torque.
        'motor_pass_fraction': env.torque_delivered_nms/requested if requested > 0. else None,
        'loop_out': env.reason == 'crash:loop_out',
        'endo': env.reason == 'crash:endo',
        'numerically_valid': env.numerically_valid,
        'max_energy_residual_ratio': env.max_energy_residual_ratio,
        # model_valid / first_model_violation / counts keep out-of-scope runs
        # separable from real outcomes.
        'model_status': env.sim.physical.model_status.as_dict(),
        'wheelie': m,  # includes wheelie_episode_records
    }
