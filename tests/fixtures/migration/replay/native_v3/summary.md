# Run recording

Run UUID: `b3b40375-bb30-486f-8930-4bbb213d37be`

Started (UTC): `2026-10-14 21:39:15.236137+02:00`. Backend: `python`. Mode: `research_headless`.

## Invocation and provenance

Evidence: `runs.metadata_json`; per-episode setup identity is in `episodes.summary_json`.

```json
{
  "check_policy": "diagnostic",
  "execution": {
    "backend": "python",
    "build_context": null,
    "extension_sha256": null,
    "mujoco_library_sha256": null,
    "mujoco_version": "3.12.0",
    "native_source_sha256": null,
    "python_source_sha256": "9f976759d8baa87e1f7096b19bdf5bdbd5bfa1dc11931824187a637681dbc92d",
    "runtime_schema": 1
  },
  "invocation": [
    "bike-research",
    "--fixture"
  ],
  "monotonic_origin_s": 1040927.533805916,
  "setup": {
    "calibration_status": "parameterized_unvalidated",
    "component_provenance": {
      "battery": "synthetic",
      "chainstay": "synthetic",
      "crank_pedals": "synthetic",
      "fork_lowers": "synthetic",
      "frame_structure": "synthetic",
      "front_wheel": "synthetic",
      "motor": "synthetic",
      "rear_wheel": "synthetic",
      "rocker": "synthetic",
      "saddle_post": "synthetic",
      "seatstay": "synthetic",
      "shock_damper": "synthetic",
      "shock_yoke": "synthetic",
      "stanchions": "synthetic",
      "steer_assembly": "synthetic"
    },
    "configuration_sha256": "af7b029188f97cf522c0e19e393745261cf3e027cee27074fa611034e4998dbf",
    "controller_interval_s": 0.005,
    "drive_mode": "articulated_effort",
    "environment_contract": {
      "declared_requirements_satisfied": true,
      "locked_environment_verified": true,
      "locked_versions_match": true,
      "packages": {
        "duckdb": {
          "declared_satisfied": true,
          "installed_version": "1.5.6",
          "locked_match": true,
          "locked_version": "1.5.6",
          "requirement": "duckdb==1.5.6"
        },
        "glfw": {
          "declared_satisfied": true,
          "installed_version": "2.10.2",
          "locked_match": true,
          "locked_version": "2.10.2",
          "requirement": "glfw>=2.10.0"
        },
        "matplotlib": {
          "declared_satisfied": true,
          "installed_version": "3.11.1",
          "locked_match": true,
          "locked_version": "3.11.1",
          "requirement": "matplotlib>=3.7.0"
        },
        "mujoco": {
          "declared_satisfied": true,
          "installed_version": "3.12.0",
          "locked_match": true,
          "locked_version": "3.12.0",
          "requirement": "mujoco>=3.0.0"
        },
        "numba": {
          "declared_satisfied": true,
          "installed_version": "0.68.0",
          "locked_match": true,
          "locked_version": "0.68.0",
          "requirement": "numba>=0.68"
        },
        "numpy": {
          "declared_satisfied": true,
          "installed_version": "2.5.2",
          "locked_match": true,
          "locked_version": "2.5.2",
          "requirement": "numpy>=1.24.0"
        },
        "plotly": {
          "declared_satisfied": true,
          "installed_version": "7.1.0",
          "locked_match": true,
          "locked_version": "7.1.0",
          "requirement": "plotly>=5.24"
        },
        "pyarrow": {
          "declared_satisfied": true,
          "installed_version": "26.0.0",
          "locked_match": true,
          "locked_version": "26.0.0",
          "requirement": "pyarrow==26.0.0"
        },
        "pytest": {
          "declared_satisfied": true,
          "installed_version": "9.1.1",
          "locked_match": true,
          "locked_version": "9.1.1",
          "requirement": "pytest>=7.0.0"
        },
        "pytz": {
          "declared_satisfied": true,
          "installed_version": "2026.5",
          "locked_match": true,
          "locked_version": "2026.5",
          "requirement": "pytz>=2026.5"
        },
        "rich": {
          "declared_satisfied": true,
          "installed_version": "15.0.0",
          "locked_match": true,
          "locked_version": "15.0.0",
          "requirement": "rich>=15.0.0"
        },
        "scipy": {
          "declared_satisfied": true,
          "installed_version": "1.17.0",
          "locked_match": true,
          "locked_version": "1.17.0",
          "requirement": "scipy==1.17.0"
        }
      },
      "platform": "macOS-27.0.1-arm64-arm-64bit-Mach-O",
      "pyproject_sha256": "0c3b0bb0ee1d1d4df3d81850cef1e6897859099e3e1e30f8bfaf49784a34cd0f",
      "python_requirement": ">=3.13",
      "python_satisfied": true,
      "python_version": "3.14.3",
      "scope": "Direct project dependencies and Python requirement; not transitive wheel integrity.",
      "uv_lock_sha256": "706c2e5cccf6e582f0adbd18aa5951a942e14a470d32452b04f95d50914807d6"
    },
    "external_speed_controller": false,
    "limitations": [
      "planar X-Z dynamics",
      "synthetic material and drive parameters",
      "single equivalent tire support; multi-support peaks not calibrated",
      "native contact work is a solver diagnostic, not material tire hysteresis"
    ],
    "model_commit": "4667639c6b2917d1aa1f65ff159d45c90544421d",
    "model_source_dirty": false,
    "model_source_sha256": "9f976759d8baa87e1f7096b19bdf5bdbd5bfa1dc11931824187a637681dbc92d",
    "physics_revision": "physical-2026-09-29",
    "resolved_config": {
      "active_suspension": {
        "coil": {
          "bumper_length_mm": 10.0,
          "bumper_peak_n": 7000.0,
          "preload_mm": 0.0,
          "rate_n_m": 114600.0,
          "stroke_mm": 65.0
        },
        "fork_damper": {
          "c_hsc": 333.21257864384114,
          "c_lsc": 634.198904787903,
          "c_reb": 909.414284430181,
          "frac_hsc": 0.5,
          "frac_lsc": 0.5,
          "frac_reb": 0.5294117647058824
        },
        "fork_pressure_psi": 85.2,
        "fork_tokens": 2,
        "shock_damper": {
          "c_hbo": 9685.766774493648,
          "c_hsc": 900.6252313054151,
          "c_lsc": 1596.0084671604288,
          "c_reb": 3990.454499701241,
          "frac_hbo": 0.5,
          "frac_hsc": 0.5,
          "frac_lsc": 0.5,
          "frac_reb": 0.5,
          "lockout_firm": false
        }
      },
      "allocation_numerical_guards": {
        "grip_radius_fraction": 1e-06,
        "support_force_n": 1e-05,
        "support_moment_nm": 1e-06
      },
      "compiled_contact_solver": {
        "front": {
          "condim": 3,
          "friction": [
            1.1,
            0.005,
            0.0001
          ],
          "native_enabled": false,
          "solimp": [
            0.9,
            0.95,
            0.001,
            0.5,
            2.0
          ],
          "solref": [
            -130000.0,
            -800.0
          ],
          "units": "MuJoCo solver parameters; not tire material stiffness or damping"
        },
        "rear": {
          "condim": 3,
          "friction": [
            1.1,
            0.005,
            0.0001
          ],
          "native_enabled": false,
          "solimp": [
            0.9,
            0.95,
            0.001,
            0.5,
            2.0
          ],
          "solref": [
            -130000.0,
            -800.0
          ],
          "units": "MuJoCo solver parameters; not tire material stiffness or damping"
        }
      },
      "compiled_equality_solimp": [
        [
          0.99,
          0.999,
          0.0001,
          0.5,
          2.0
        ],
        [
          0.99,
          0.999,
          0.0001,
          0.5,
          2.0
        ],
        [
          0.9,
          0.95,
          0.001,
          0.5,
          2.0
        ],
        [
          0.9,
          0.95,
          0.001,
          0.5,
          2.0
        ],
        [
          0.9,
          0.95,
          0.001,
          0.5,
          2.0
        ],
        [
          0.9,
          0.95,
          0.001,
          0.5,
          2.0
        ],
        [
          0.9,
          0.95,
          0.001,
          0.5,
          2.0
        ]
      ],
      "compiled_equality_solref": [
        [
          0.0025,
          1.0
        ],
        [
          0.0025,
          1.0
        ],
        [
          0.0025,
          1.0
        ],
        [
          0.0025,
          1.0
        ],
        [
          0.0025,
          1.0
        ],
        [
          0.0025,
          1.0
        ],
        [
          0.0025,
          1.0
        ]
      ],
      "geometry_and_suspension": {
        "bb_drop": 22.5,
        "crank_length": 165.0,
        "effective_seat_angle_deg": 77.0,
        "fork_air_tokens": 2,
        "fork_damping": 450.0,
        "fork_hsc": 2,
        "fork_initial_psi": 85.2,
        "fork_lsc": 7,
        "fork_offset": 44.0,
        "fork_rebound": 9,
        "fork_stiffness": 6000.0,
        "fork_travel": 180.0,
        "front_wheel_radius": 372.0,
        "head_angle_deg": 64.0,
        "headtube_length": 120.0,
        "reach": 480.0,
        "rear_wheel_radius": 352.0,
        "rear_wheel_travel": 180.0,
        "shock_damping": 1200.0,
        "shock_eye_to_eye": 205.0,
        "shock_hbo": 2,
        "shock_hsc": 2,
        "shock_lockout": false,
        "shock_lsc": 7,
        "shock_rebound": 7,
        "shock_stiffness": 114600.0,
        "shock_stroke": 65.0,
        "stack": 646.0,
        "wheel_radius": 372.0,
        "wheelbase": 1280.55
      },
      "joint_envelope_sha256": "4e9cacbce5b7a39d369f3e1d7eacedbef5386c56e1707045ea140621baa43f64",
      "joint_envelope_status": "declared_unvalidated",
      "joint_strength_sha256": "0921312611e18611da3e384d2827e63c183eb642f0b9be1cbbf0a58480eee5f8",
      "joint_strength_status": "declared_unvalidated",
      "mass_budget": {
        "battery_mass": 4.3,
        "chainstay_mass": 1.25,
        "crank_pedals_mass": 0.85,
        "fork_lowers_mass": 1.5,
        "frame_structure_mass": 3.2,
        "front_wheel_mass": 2.4,
        "motor_mass": 2.9,
        "rear_wheel_mass": 2.8,
        "rocker_mass": 0.4,
        "saddle_post_mass": 0.9,
        "seatstay_mass": 1.05,
        "shock_damper_mass": 0.6,
        "shock_yoke_mass": 0.3,
        "stanchions_mass": 0.85,
        "steer_assembly_mass": 1.1,
        "wheel_hub_core_fraction": 0.25,
        "wheel_rim_tire_fraction": 0.75
      },
      "pedal_support_half_length_m": {
        "front": 0.05,
        "rear": 0.05
      },
      "physics": {
        "articulated": {
          "activation_tau_s": 0.0,
          "active_positive_power_limit_w": 450.0,
          "arm_reach_fraction": 0.92,
          "balance_dwell_s": 0.5,
          "balance_floor_kmh": 4.0,
          "balance_grace_s": 3.0,
          "bar_support_fraction": 0.12,
          "coasting_brake_d_nm_s_rad": 2.0,
          "coasting_brake_limit_nm": 60.0,
          "coupled_task_control": false,
          "foot_mu": 0.9,
          "grip_attachment": "connect",
          "grip_c_ns_m": 150.0,
          "grip_capture_distance_m": 0.02,
          "grip_capture_speed_mps": 0.2,
          "grip_k_n_m": 4000.0,
          "grip_pair_force_limit_n": null,
          "grip_pull_per_hand_n": 300.0,
          "grip_release_distance_m": 0.12,
          "joint_envelope_path": "/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp/tests/fixtures/migration/config/rider_joint_envelope_anatomical.json",
          "joint_envelope_soft_k_nm_rad": 100.0,
          "joint_envelope_soft_margin_rad": 0.1,
          "joint_kd_nms_rad": 15.0,
          "joint_kp_nm_rad": 600.0,
          "joint_limit_nm": 100.0,
          "joint_passive_damping_nms_rad": 0.0,
          "joint_power_limit_w": 250.0,
          "joint_speed_limit_rad_s": 20.0,
          "joint_strength_path": "/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp/tests/fixtures/migration/config/rider_strength_reference.json",
          "link_max_gap_m": 0.005,
          "pedal_ankle_amplitude_rad": 0.1,
          "pedal_attachment": "spindle",
          "pedal_c_ns_m": 100.0,
          "pedal_min_normal_n": 20.0,
          "pedal_patch_half_length_m": 0.025,
          "pedal_scrape_fraction": 0.5,
          "pedal_support_fraction": 0.33,
          "pedal_torque_ripple": 0.5,
          "posture_pitch_d_nms_rad": 60.0,
          "posture_pitch_k_nm_rad": 600.0,
          "posture_pitch_limit_nm": 60.0,
          "posture_sole_depth_m": 0.003,
          "posture_translation_d_ns_m": 150.0,
          "posture_translation_k_n_m": 1500.0,
          "posture_translation_limit_n": 400.0,
          "return_foot_preload_n": 40.0,
          "road_lookahead_m": 2.0,
          "saddle_attachment": "pin",
          "saddle_mu": 0.6,
          "saddle_patch_half_length_m": 0.045,
          "saddle_reserve_weight_fraction": 0.15,
          "stance_blend_load_n": 50.0,
          "support_c_ns_m": 500.0,
          "support_k_n_m": 100000.0,
          "support_length_m": 0.1,
          "support_mu": 0.8,
          "support_pad_radius_m": 0.02,
          "support_tangent_k_n_m": 20000.0,
          "swing_clearance_m": 0.003
        },
        "closure_time_constant_s": 0.0025,
        "control_period_s": 0.005,
        "drive": {
          "assist": {
            "cutoff_mps": 6.944444444444445,
            "engage_torque_nm": 4.0,
            "gain": 2.0,
            "gate_min_crank_rad_s": 0.08726646259971647,
            "max_power": 500.0,
            "max_torque": 80.0,
            "mode": "turbo",
            "profile": "bosch_cx_gen4",
            "slew": 400.0,
            "taper_width_mps": 0.5555555555555556,
            "tau": 0.05,
            "torque_curve": [
              [
                0.0,
                85.0
              ],
              [
                120.0,
                85.0
              ],
              [
                120.1,
                47.7
              ],
              [
                180.0,
                0.0
              ]
            ]
          },
          "battery": {
            "copper_w_per_nm2": 0.02,
            "enabled": false,
            "energy_j": 1800000.0,
            "idle_w": 5.0,
            "speed_w_per_rad_s2": 0.0
          },
          "bearing_c_nms_rad": 0.03,
          "brake_ceiling_nm": 200.0,
          "chain_c_ns_m": 10.0,
          "chain_k_n_m": 200000.0,
          "crank_phase_rad": 0.0,
          "freehub_c_nms_rad": 0.5,
          "freehub_k_nm_rad": 1000.0,
          "gearing": {
            "chain_pitch_m": 0.0127,
            "front_teeth": 22,
            "rear_teeth": 51
          },
          "human_torque_nm": 0.0,
          "motor_clutch": false,
          "pedaling": {
            "coast_above_rpm": 120.0,
            "coast_cadence_tau_s": 0.35,
            "effort_slew_nm_s": 300.0,
            "enabled": true,
            "mash_cadence_rpm": 45.0,
            "mash_torque_nm": 60.0,
            "resume_below_rpm": 105.0,
            "rollback_brake": true,
            "rollback_demand": 1.0,
            "rollback_engage_mps": 0.25,
            "rollback_release_mps": 0.0,
            "stop_time_s": 0.35
          },
          "rotor_inertia_kgm2": 0.0,
          "shifting": {
            "cadence_smoothing_tau_s": 0.35,
            "cassette": [
              10,
              12,
              14,
              16,
              18,
              21,
              24,
              28,
              33,
              39,
              45,
              51
            ],
            "enabled": true,
            "shift_cooldown_s": 0.4,
            "shift_cut_duration_s": 0.2,
            "target_cadence_max_rpm": 110.0,
            "target_cadence_min_rpm": 75.0,
            "torque_factor": 0.3,
            "upshift_slip_limit_mps": 0.5,
            "upshift_slip_mode": "legacy_signed"
          },
          "torque_ripple": 0.35,
          "transmission_model": "ideal_mid_drive"
        },
        "drive_mode": "articulated_effort",
        "end_stops": {
          "damping_n_s_m": 500.0,
          "overtravel_m": 0.01,
          "provenance": "synthetic",
          "reference_deflection_m": 0.01,
          "reference_force_n": 7000.0
        },
        "equilibrium_cache_enabled": true,
        "equilibrium_refine_after_s": 1.0,
        "equilibrium_refine_period_s": 3.0,
        "equilibrium_state_file": null,
        "initial_front_brake": 1.0,
        "initial_rear_brake": 1.0,
        "initial_speed_mps": 0.0,
        "physics_mode": "physical",
        "pitch_assist": false,
        "resistance": {
          "cda_m2": 0.5,
          "crr": 0.015,
          "point_body_m": [
            0.0,
            0.0,
            0.5
          ],
          "rho_kg_m3": 1.2,
          "rolling_taper_rad_s": 0.2,
          "wind_world_mps": [
            0.0,
            0.0,
            0.0
          ]
        },
        "seated_climb": {
          "enabled": true,
          "front_load_share_target": 0.3,
          "lean_gain": 1.0,
          "lean_rate_rad_s": 0.5,
          "lean_trim_gain_rad_s": 0.3,
          "lean_trim_limit_rad": 0.15,
          "max_backward_lean_rad": 0.1,
          "max_crank_torque_nm": 60.0,
          "max_forward_lean_rad": 0.35,
          "orientation_tau_s": 0.5,
          "period_s": 0.01,
          "reaction_delay_s": 0.15,
          "surge_budget_s": 15.0,
          "surge_grade": 0.2,
          "surge_power_w": 400.0,
          "surge_recovery_rate": 0.3333333333333333,
          "target_crank_power_w": 250.0,
          "torque_slew_nm_s": 300.0,
          "trim_dead_time_s": 0.2
        },
        "timestep_s": 0.00125,
        "tires": {
          "backend": "compliant_2d",
          "distinct_normal_deg": 20.0,
          "distributed": {
            "calibration_status": "experimental_unvalidated",
            "density": {
              "damping_ns_m": 2000.0,
              "knots_m": [
                0.0,
                0.004,
                0.012
              ],
              "provenance": "synthetic density; not fitted or measured",
              "stiffness_n_m": [
                300000.0,
                0.0,
                0.0
              ]
            },
            "fitting_dataset_id": "none-synthetic",
            "front_material": null,
            "rear_material": null,
            "station_count": 128,
            "valid_deflection_range_m": [
              0.0,
              0.04
            ],
            "valid_load_range_n": [
              0.0,
              2000.0
            ]
          },
          "front": {
            "material": {
              "pressure_pa_gauge": 200000.0,
              "provenance": "synthetic",
              "radial_c_ns_m": 800.0,
              "radial_k_n_m": 130000.0,
              "valid_load_range_n": [
                0.0,
                2000.0
              ]
            },
            "mu": 1.1,
            "relaxation_length_m": 0.2,
            "tangent_k_n_m": 20000.0
          },
          "pneumatic": {
            "front": {
              "area_factor": 0.4435,
              "carcass_stiffness_n_mm2": 0.1194,
              "casing_width_mm": 60.0,
              "compressed_casing_mm": 6.0,
              "contact_length_factor": 0.8589,
              "crr_reference": 0.011,
              "hysteresis_rate_eps_mps": 0.01,
              "loss_factor": 0.0662,
              "name": "Schwalbe Magic Mary 29x2.4",
              "outer_radius_mm": 372.0,
              "pressure_bar": 1.5,
              "rate_relaxation_s": 0.2,
              "rate_stiffening": 0.2484,
              "relaxation_length_mm": 60.0,
              "rim_radius_mm": 320.0,
              "rim_stiffness_n_mm2": 28.0,
              "section_height_mm": 57.0,
              "tread_loss_crr": 0.00586
            },
            "rear": {
              "area_factor": 0.4576,
              "carcass_stiffness_n_mm2": 0.1225,
              "casing_width_mm": 60.0,
              "compressed_casing_mm": 6.0,
              "contact_length_factor": 0.8801,
              "crr_reference": 0.0103,
              "hysteresis_rate_eps_mps": 0.01,
              "loss_factor": 0.069,
              "name": "Schwalbe Hans Dampf 27.5x2.4",
              "outer_radius_mm": 352.0,
              "pressure_bar": 1.7,
              "rate_relaxation_s": 0.2,
              "rate_stiffening": 0.2498,
              "relaxation_length_mm": 60.0,
              "rim_radius_mm": 300.0,
              "rim_stiffness_n_mm2": 28.0,
              "section_height_mm": 55.0,
              "tread_loss_crr": 0.00481
            },
            "surface": null,
            "tier": "fast"
          },
          "rear": {
            "material": {
              "pressure_pa_gauge": 200000.0,
              "provenance": "synthetic",
              "radial_c_ns_m": 800.0,
              "radial_k_n_m": 130000.0,
              "valid_load_range_n": [
                0.0,
                2000.0
              ]
            },
            "mu": 1.1,
            "relaxation_length_m": 0.2,
            "tangent_k_n_m": 20000.0
          },
          "significance_fraction": 0.05,
          "significant_delta_m": 0.0001,
          "surface_mode": "track"
        }
      },
      "rider": {
        "arm_resonance_hz": 4.0,
        "bar_share": 0.12,
        "damping_ratio": 0.4,
        "height_m": 1.8,
        "helmet_mass_kg": 0.4,
        "inseam_m": null,
        "leg_resonance_hz": 5.0,
        "legs": "rigid",
        "mass_kg": 80.0,
        "pedal_share": 0.33,
        "saddle_interface_c_ns_m": 2762.0,
        "saddle_interface_k_n_m": 101000.0,
        "saddle_share": 0.55,
        "torso_resonance_hz": 7.0,
        "variant": "articulated_planar"
      },
      "seed": 210,
      "target_speed_kmh": null,
      "terrain_sha256": "d9326d33e13d7c2574a8995baa3408dd625ee378209e0de4f8397db1dbe16928",
      "track": {
        "description": "No obstacles; sag, rolling resistance and coast-down reference.",
        "length_m": 112.0,
        "name": "flat",
        "obstacles": [],
        "surface": "hardpack"
      }
    },
    "rider_model": "articulated_planar",
    "schema_version": 2,
    "seed": 210,
    "terrain_sha256": "d9326d33e13d7c2574a8995baa3408dd625ee378209e0de4f8397db1dbe16928",
    "timestep_s": 0.00125,
    "tire_backend": "compliant_2d",
    "versions": {
      "mujoco": "3.12.0",
      "numpy": "2.5.2",
      "python": "3.14.3",
      "scipy": "1.17.0"
    }
  }
}
```

## Interpretation and limits

This is saved numerical evidence, not a claim that the model is calibrated. Recording retains the launch in RAM and publishes at finalization; crashes, SIGKILL and out-of-memory termination have no durability guarantee. NULL, absence and nonfinite values are separate states. No cause is inferred from temporal coincidence.

A forwarded endpoint measurement is not an instantaneous peak at every interval. The extrema below exclude forwarded energy rows; cumulative fields are reported as final values and are not summed. Missing/nonfinite coverage is reported explicitly.

## Episode 0

Native generation: `0`. Physical prefix: `{"first_interval":0,"first_time_s":0.0,"last_interval":79,"last_time_s":0.10000000000000006,"rows":80}`.

Completed control transitions: `10`. Checked research replay eligible: `True` (contract `3`).

### Outcome, validity and first failure

Evidence: `episodes.summary_json`, selected by `episode_id`.

```json
{
  "execution_changed_during_run": null,
  "failure": null,
  "first_failure": [
    0.005,
    [
      "energy.constraint_work"
    ]
  ],
  "model_status": {
    "calibration_status": "parameterized_unvalidated",
    "first_model_violation": {
      "interval_id": 3,
      "reasons": [
        "energy.constraint_work"
      ],
      "time_s": 0.00375
    },
    "model_valid": false,
    "model_violation_counts": {
      "energy.constraint_work": 20,
      "foot_rear.normal": 2
    },
    "numerically_valid": true
  },
  "outcome": {
    "balance_lost": false,
    "balance_lost_at_m": null,
    "crashed": false,
    "position_m": 2.0121401599295443,
    "reason": "duration",
    "steps": 80,
    "time_s": 0.10000000000000006
  },
  "source_changed_during_run": null,
  "tail_failure": null
}
```

### Important events

Source events keep their recorded classification. `derived_transition` events describe retained channel changes under a named rule; they are not controller explanations.

Clock semantics: `received_wall_s` is the monotonic receipt timestamp (a wall clock, not simulation time); `interval_time_s` is the simulation start time of the interval the input was processed on, and is null when no interval carries that `processed_step`.

| Event | Kind | Received wall (s) | Sim time (s) | Processed step | Applied step | Recorded detail |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | episode_started | 1040927.533955083 | 0.0 | 0 | unavailable / null | {"capture_mode":"full","check_policy":"diagnostic","time_s":0.0} |
| 1 | derived_transition | 1040927.7937275 | 0.00375 | 3 | 3 | {"changes":{"model_valid":false,"model_violation_counts":{"energy.constraint_work":1}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":3} |
| 2 | derived_transition | 1040927.7937275 | 0.00375 | 3 | 3 | {"changes":{"first_model_violation":{"interval_id":3,"reasons":["energy.constraint_work"],"time_s":0.00375}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_first_failure","interval_id":3} |
| 3 | derived_transition | 1040927.7937275 | 0.00875 | 7 | 7 | {"changes":{"model_violation_counts":{"energy.constraint_work":2}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":7} |
| 4 | derived_transition | 1040927.7937275 | 0.013749999999999998 | 11 | 11 | {"changes":{"model_violation_counts":{"energy.constraint_work":3}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":11} |
| 5 | derived_transition | 1040927.7937275 | 0.01875 | 15 | 15 | {"changes":{"model_violation_counts":{"energy.constraint_work":4}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":15} |
| 6 | derived_transition | 1040927.7937275 | 0.023750000000000004 | 19 | 19 | {"changes":{"model_violation_counts":{"energy.constraint_work":5}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":19} |
| 7 | derived_transition | 1040927.7937275 | 0.028750000000000008 | 23 | 23 | {"changes":{"model_violation_counts":{"energy.constraint_work":6}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":23} |
| 8 | derived_transition | 1040927.7937275 | 0.03375000000000001 | 27 | 27 | {"changes":{"model_violation_counts":{"energy.constraint_work":7}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":27} |
| 9 | derived_transition | 1040927.7937275 | 0.038750000000000014 | 31 | 31 | {"changes":{"model_violation_counts":{"energy.constraint_work":8}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":31} |
| 10 | derived_transition | 1040927.7937275 | 0.04375000000000002 | 35 | 35 | {"changes":{"model_violation_counts":{"energy.constraint_work":9}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":35} |
| 11 | derived_transition | 1040927.7937275 | 0.04875000000000002 | 39 | 39 | {"changes":{"model_violation_counts":{"energy.constraint_work":10}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":39} |
| 12 | derived_transition | 1040927.7937275 | 0.05375000000000003 | 43 | 43 | {"changes":{"model_violation_counts":{"energy.constraint_work":11}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":43} |
| 13 | derived_transition | 1040927.7937275 | 0.05875000000000003 | 47 | 47 | {"changes":{"model_violation_counts":{"energy.constraint_work":12}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":47} |
| 14 | derived_transition | 1040927.7937275 | 0.06375000000000003 | 51 | 51 | {"changes":{"model_violation_counts":{"energy.constraint_work":13}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":51} |
| 15 | derived_transition | 1040927.7937275 | 0.06875000000000003 | 55 | 55 | {"changes":{"model_violation_counts":{"energy.constraint_work":14}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":55} |
| 16 | derived_transition | 1040927.7937275 | 0.07375000000000004 | 59 | 59 | {"changes":{"model_violation_counts":{"energy.constraint_work":15}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":59} |
| 17 | derived_transition | 1040927.7937275 | 0.07875000000000004 | 63 | 63 | {"changes":{"model_violation_counts":{"energy.constraint_work":16}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":63} |
| 18 | derived_transition | 1040927.7937275 | 0.08375000000000005 | 67 | 67 | {"changes":{"model_violation_counts":{"energy.constraint_work":17}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":67} |
| 19 | derived_transition | 1040927.7937275 | 0.08875000000000005 | 71 | 71 | {"changes":{"model_violation_counts":{"energy.constraint_work":18}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":71} |
| 20 | derived_transition | 1040927.7937275 | 0.09375000000000006 | 75 | 75 | {"changes":{"model_violation_counts":{"energy.constraint_work":19}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":75} |
| 21 | derived_transition | 1040927.7937275 | 0.09750000000000006 | 78 | 78 | {"changes":{"model_violation_counts":{"energy.constraint_work":19,"foot_rear.normal":1}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":78} |
| 22 | derived_transition | 1040927.7937275 | 0.09875000000000006 | 79 | 79 | {"changes":{"model_violation_counts":{"energy.constraint_work":20,"foot_rear.normal":2}},"classification":"derived_transition","derivation_rule":"channel_value_change.v1","derived_kind":"model_validity","interval_id":79} |
| 23 | model_first_failure | 1040927.7937275 | unavailable / null | 80 | unavailable / null | {"classification":"measured","evidence":[0.005,["energy.constraint_work"]]} |
| 24 | episode_ended | 1040927.7937275 | unavailable / null | 80 | unavailable / null | {"processed_wall_s":1040927.7937275,"reason":"duration","time_s":0.10000000000000006} |

### Scalar coverage and extrema

| Field | Unit / phase | Finite | Missing | Nonfinite | Minimum / final | Maximum |
| --- | --- | --- | --- | --- | --- | --- |
| `channel_control.crank_target_rate_rad_s` | rad/s / control_period | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_control.human_torque_nm` | N·m / control_period | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_control.motor_limit_nm` | N·m / control_period | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_control.motor_torque_nm` | N·m / control_period | 4 | 76 | 0 | 0.0 | 0.0 |
| `channel_drive.assist_gain` | 1 / interval | 80 | 0 | 0 | 3.4 | 3.4 |
| `channel_drive.assist_sensor_nm` | N·m / interval_start | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.battery_energy_j` | J / interval | 80 | 0 | 0 | 1800000.0 | 1800000.0 |
| `channel_drive.cadence_rpm` | rpm / interval_start | 80 | 0 | 0 | 0.0 | 2.5025813465888476 |
| `channel_drive.chain_dissipation_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.chain_energy_j` | J / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.chain_extension_m` | m / interval_start | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.chain_extension_rate_mps` | m/s / interval_start | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.chain_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.chain_tension_n` | N / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.crank_clutch_dissipation_power_w` | W / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.crank_clutch_torque_nm` | N·m / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.crank_phase_rad` | rad / interval_start | 80 | 0 | 0 | 0.0049819198827646995 | 0.021974860166581667 |
| `channel_drive.crank_rad_s` | rad/s / interval_start | 80 | 0 | 0 | 0.0 | 0.2620697057818125 |
| `channel_drive.crank_target_phase_rad` | rad / control_period | 80 | 0 | 0 | 0.0049819198827646995 | 0.0049819198827646995 |
| `channel_drive.crank_target_rate_rad_s` | rad/s / control_period | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.drive_shaft_rad_s` | rad/s / interval_start | 80 | 0 | 0 | 0.0 | 0.2620697057818125 |
| `channel_drive.electrical_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.freehub_deflection_rad` | rad / interval_start | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.freehub_dissipation_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.freehub_energy_j` | J / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.freehub_power_w` | W / interval | 80 | 0 | 0 | -0.08787071382726785 | 0.08538458016895825 |
| `channel_drive.freehub_torque_nm` | N·m / interval | 80 | 0 | 0 | 3.152220288509296 | 11.565005022142438 |
| `channel_drive.front_brake_demand` | 1 / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.front_brake_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.front_brake_torque_nm` | N·m / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.gear_ratio` | 1 / interval | 80 | 0 | 0 | 0.43137254901960786 | 0.43137254901960786 |
| `channel_drive.human_active_power_w` | W / interval | 80 | 0 | 0 | -68.51437490218275 | 52.99847813671324 |
| `channel_drive.human_command_nm` | N·m / control_period | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.human_joint_power_w` | W / interval | 80 | 0 | 0 | -68.51437490218275 | 52.99847813671324 |
| `channel_drive.human_sensor_nm` | N·m / interval_start | 80 | 0 | 0 | 1.4352952049923613 | 4.872076443036668 |
| `channel_drive.human_setpoint_nm` | N·m / control_period | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.human_torque_nm` | N·m / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.motor_freewheel_dissipation_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.motor_freewheel_torque_nm` | N·m / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.motor_limit_nm` | N·m / control_period | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.motor_limited_request_nm` | N·m / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.motor_request_nm` | N·m / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.motor_setpoint_nm` | N·m / control_period | 4 | 76 | 0 | 0.0 | 0.0 |
| `channel_drive.motor_shaft_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.motor_torque_nm` | N·m / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.rear_brake_demand` | 1 / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.rear_brake_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.rear_brake_torque_nm` | N·m / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.required_cadence_rpm` | rpm / interval_start | 80 | 0 | 0 | 0.0 | 2.5845563897469153 |
| `channel_drive.rollback_brake_demand` | 1 / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_drive.shift_constraint_work_cumulative_j` | J / episode | 0 | 80 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_drive.shift_interval_constraint_work_j` | J / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.shift_parameter_work_j` | J / episode | 0 | 80 | 0 | unavailable / null | "cumulative: not a peak" |
| `channel_drive.shift_time_s` | s / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.shift_torque_factor` | 1 / interval | 80 | 0 | 0 | 1.0 | 1.0 |
| `channel_drive.transmission_boundary_m` | m / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_constraint_defect_m` | m / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_gap_m` | m / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_interval_work_j` | J / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_phi_m` | m / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_reaction_error_n` | N / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_drive.transmission_tension_n` | N / interval | 0 | 80 | 0 | unavailable / null | unavailable / null |
| `channel_energy.active_work_j` | J / episode | 80 | 0 | 0 | -0.07261688714663364 | "cumulative: not a peak" |
| `channel_energy.constraint_absolute_j` | J / episode | 80 | 0 | 0 | 0.04797601322007436 | "cumulative: not a peak" |
| `channel_energy.constraint_signed_j` | J / episode | 80 | 0 | 0 | -0.014562070018710864 | "cumulative: not a peak" |
| `channel_energy.constraint_work_ratio` | 1 / episode | 73 | 7 | 0 | 0.2553150302062784 | "cumulative: not a peak" |
| `channel_energy.electrical_residual_j` | J / episode | 80 | 0 | 0 | 0.0 | "cumulative: not a peak" |
| `channel_energy.electrical_work_j` | J / episode | 80 | 0 | 0 | 0.0 | "cumulative: not a peak" |
| `channel_energy.energy_scale_j` | J / episode | 80 | 0 | 0 | 26.079533545992142 | "cumulative: not a peak" |
| `channel_energy.external_work_j` | J / episode | 80 | 0 | 0 | -0.011614796190591106 | "cumulative: not a peak" |
| `channel_energy.loss_j` | J / episode | 80 | 0 | 0 | 0.02392297764010526 | "cumulative: not a peak" |
| `channel_energy.loss_step_j` | J / interval | 80 | 0 | 0 | 0.0 | 0.0037663491341720454 |
| `channel_energy.mechanical_energy_j` | J / interval_end | 21 | 0 | 0 | 653.6322702634268 | 653.7422109899826 |
| `channel_energy.motor_positive_j` | J / episode | 80 | 0 | 0 | 0.0 | "cumulative: not a peak" |
| `channel_energy.motor_signed_j` | J / episode | 80 | 0 | 0 | 0.0 | "cumulative: not a peak" |
| `channel_energy.muscle_positive_j` | J / episode | 80 | 0 | 0 | 0.18790908307009102 | "cumulative: not a peak" |
| `channel_energy.muscle_signed_j` | J / episode | 80 | 0 | 0 | -0.07261688714663364 | "cumulative: not a peak" |
| `channel_energy.residual_j` | J / interval_end | 21 | 0 | 0 | -0.0017787840787240163 | 0.1944230494859726 |
| `channel_energy.solver_constraint_work_j` | J / episode | 80 | 0 | 0 | -0.014562070018710864 | "cumulative: not a peak" |
| `channel_energy.source_positive_work_j` | J / episode | 80 | 0 | 0 | 0.18790908307009102 | "cumulative: not a peak" |
| `channel_energy.source_time_s` | s / interval | 80 | 0 | 0 | 0.00125 | 0.10000000000000006 |
| `channel_mass.gravitational_energy_j` | J / interval_start | 21 | 59 | 0 | 627.5380320194832 | 627.5803734001925 |
| `channel_mass.kinetic_energy_j` | J / interval_start | 21 | 59 | 0 | 0.0 | 0.05789105227900562 |
| `channel_mass.mass_kg` | kg / interval_start | 80 | 0 | 0 | 104.39999999999999 | 104.39999999999999 |
| `channel_rider.rider_active_positive_power_limit_w` | W / interval | 80 | 0 | 0 | 450.0 | 450.0 |
| `channel_rider.rider_lean_limit_rad` | rad / control_period | 80 | 0 | 0 | 0.5671875000000001 | 0.5671875000000001 |
| `channel_rider.rider_passive_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_rider.rider_passive_work_step_j` | J / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_rider.rider_positive_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 53.32904566746713 |
| `channel_rider.rider_positive_work_step_j` | J / interval | 80 | 0 | 0 | 0.0 | 0.0666613070843339 |
| `channel_rider.weld_crank_torque_nm` | N·m / interval | 21 | 59 | 0 | 1.4385257884740728 | 4.92553458310275 |
| `channel_sensors.encoder_crank_rad_s` | rad/s / interval_start | 80 | 0 | 0 | 0.0 | 0.2620697057818125 |
| `channel_sensors.encoder_front_wheel_rad_s` | rad/s / interval_start | 80 | 0 | 0 | 0.0 | 0.14211698718731705 |
| `channel_sensors.encoder_rear_wheel_rad_s` | rad/s / interval_start | 80 | 0 | 0 | 0.0 | 0.1167527542941074 |
| `channel_sensors.human_torque_nm` | N·m / interval_start | 80 | 0 | 0 | 1.4352952049923613 | 4.872076443036668 |
| `channel_sensors.motor_torque_nm` | N·m / interval_start | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_suspension.fork_travel_m` | m / interval_start | 80 | 0 | 0 | 0.05981586201534997 | 0.060883870719743695 |
| `channel_suspension.fork_velocity_mps` | m/s / interval_start | 80 | 0 | 0 | -0.05285644934029909 | 0.01965901884214011 |
| `channel_suspension.linkage_closure_max_m` | m / interval_end | 80 | 0 | 0 | 4.453857487307644e-05 | 4.476221486848786e-05 |
| `channel_suspension.shock_solver_limit_power_w` | W / interval | 80 | 0 | 0 | 0.0 | 0.0 |
| `channel_suspension.shock_stroke_m` | m / interval_start | 80 | 0 | 0 | 0.014432671768742805 | 0.014623452880115887 |
| `channel_suspension.shock_velocity_mps` | m/s / interval_start | 80 | 0 | 0 | -0.00044644133052610486 | 0.00477176127517913 |
| `physics.dt_s` | s / interval | 80 | 0 | 0 | 0.0012499999999999942 | 0.0012500000000000011 |
| `physics.interval_end_s` | s / interval_end | 80 | 0 | 0 | 0.00125 | 0.10000000000000006 |
| `physics.time_s` | s / interval_start | 80 | 0 | 0 | 0.0 | 0.09875000000000006 |

Reproduce each extrema row as `SELECT min(FIELD), max(FIELD) FROM TABLE WHERE episode_id=0 AND isfinite(FIELD)`; for `channel_energy` fields marked `forwarded` in `field_catalog`, also require `measured IS TRUE`. For phase `episode`, select the last `interval_id` instead of aggregating cumulative values.

### Recorded component power integrals

Rule `left_rectangle_recorded_power.v1`: sum of recorded `power_w * dt_s`. This is a derived finite-subset integral when any power is missing/nonfinite; it does not replace the engine's recorded cumulative accounting.

| Component | Intervals | Finite powers | Derived work (J) |
| --- | --- | --- | --- |
| `act_rider_elbow_left` | 80 | 80 | -0.02466867154150585 |
| `act_rider_elbow_right` | 80 | 80 | -0.024668671541542873 |
| `act_rider_hip_front` | 80 | 80 | -0.0043630648676234425 |
| `act_rider_hip_rear` | 80 | 80 | 0.0 |
| `act_rider_knee_front` | 80 | 80 | -0.0021129820947169173 |
| `act_rider_knee_rear` | 80 | 80 | 0.0 |
| `act_rider_shoulder_left` | 80 | 80 | -0.006932049603267905 |
| `act_rider_shoulder_right` | 80 | 80 | -0.006932049603301219 |
| `act_rider_torso_hinge` | 80 | 80 | -0.0029393978946755175 |
| `aerodynamic` | 80 | 80 | -5.705271847855592e-08 |
| `chain` | 80 | 80 | 0.0 |
| `closure` | 80 | 80 | -0.013854331002374154 |
| `drive_bearings` | 80 | 80 | -0.00013664674719773723 |
| `engine_passive` | 80 | 80 | -6.572926442840607e-05 |
| `fork_damper` | 80 | 80 | -0.02050726912277442 |
| `fork_spring` | 80 | 80 | 0.36664682533368986 |
| `freehub` | 80 | 80 | 0.0 |
| `front_brake` | 80 | 80 | 0.0 |
| `front_static_brake` | 80 | 80 | 0.0 |
| `ideal_transmission` | 80 | 80 | -0.0007077390163367322 |
| `joint_limits` | 80 | 80 | 0.0 |
| `mid_drive` | 80 | 80 | 0.0 |
| `native_contact` | 80 | 80 | 0.0 |
| `rear_brake` | 80 | 80 | 0.0 |
| `rear_static_brake` | 80 | 80 | 0.0 |
| `rider_interfaces` | 80 | 80 | 0.0 |
| `rider_joint_envelope` | 80 | 80 | 0.0 |
| `rider_passive_damping` | 80 | 80 | 0.0 |
| `road_rolling` | 80 | 80 | -0.011614739137872641 |
| `shock_bumper` | 80 | 80 | 0.0 |
| `shock_coil` | 80 | 80 | -0.31504019065617506 |
| `shock_damper` | 80 | 80 | -0.0008107231643775725 |
| `shock_hbo` | 80 | 80 | 0.0 |
| `shock_solver_limit` | 80 | 80 | 0.0 |
| `shock_top_out` | 80 | 80 | 0.0 |
| `shock_upper_stop` | 80 | 80 | 0.0 |
| `tires` | 80 | 80 | -0.02826813114032649 |

Evidence query: `forces JOIN physics USING(episode_id,interval_id)`, `episode_id=0`, grouped by `component`.

### Episode metrics and source identity

```json
{
  "episode_metrics": {
    "balance_lost": false,
    "balance_lost_at_m": null,
    "demand_integral_nms": null,
    "duration_s": 0.10000000000000006,
    "endo": false,
    "finish_time_s": null,
    "loop_out": false,
    "max_energy_residual_ratio": 0.010620163614451995,
    "max_fork_travel_m": 0.060883870719743695,
    "max_shock_stroke_m": 0.014623452880115887,
    "mean_speed_mps": 0.014961796779622274,
    "model_status": {
      "calibration_status": "parameterized_unvalidated",
      "first_model_violation": {
        "interval_id": 3,
        "reasons": [
          "energy.constraint_work"
        ],
        "time_s": 0.00375
      },
      "model_valid": false,
      "model_violation_counts": {
        "energy.constraint_work": 20,
        "foot_rear.normal": 2
      },
      "numerically_valid": true
    },
    "motor_pass_fraction": null,
    "numerically_valid": true,
    "operator_intervention": false,
    "outcome": "duration",
    "progress_m": 0.0014961796779622283,
    "torque_delivered_nms": 0.0,
    "torque_requested_nms": 0.0,
    "wheelie": {
      "contact_state_time_s": {
        "two_wheels": 0.10000000000000006
      },
      "duration_s": 0.10000000000000006,
      "flight_time_s": 0.0,
      "front_lift_time_s": 0.0,
      "front_load_fraction_mean": 0.4084571520162067,
      "front_load_fraction_min": 0.3898128106450679,
      "front_unloaded_time_s": 0.0,
      "max_front_clearance_m": 0.0,
      "max_pitch_rate_up_rad_s": 0.06602960989192447,
      "max_relative_pitch_rad": 0.0012749698978237536,
      "min_front_load_n": 392.54880126667035,
      "rear_slip_distance_m": 0.0009360237072205245,
      "wheelie_candidate_time_s": 0.0,
      "wheelie_episode_records": [],
      "wheelie_episodes": 0,
      "wheelie_time_s": 0.0
    }
  },
  "execution": null,
  "model_source_sha256": "9f976759d8baa87e1f7096b19bdf5bdbd5bfa1dc11931824187a637681dbc92d",
  "research": {
    "actual_sensor_seed": 210,
    "actuator_delay_steps": 4,
    "capture_mode": "full",
    "config": {
      "actuator_delay_s": 0.005,
      "async_telemetry": false,
      "control_period_s": 0.01,
      "duration_s": 0.1,
      "maximum_energy_residual_ratio": 0.05,
      "record_decimation": 80,
      "seed": 210,
      "stop_on_model_violation": false,
      "wheelie_persistence_s": 0.02
    },
    "configuration_changed_during_run": false,
    "control_steps": 8,
    "error": null,
    "execution": {
      "backend": "python",
      "build_context": null,
      "extension_sha256": null,
      "mujoco_library_sha256": null,
      "mujoco_version": "3.12.0",
      "native_source_sha256": null,
      "python_source_sha256": "9f976759d8baa87e1f7096b19bdf5bdbd5bfa1dc11931824187a637681dbc92d",
      "runtime_schema": 1
    },
    "execution_changed_during_run": false,
    "max_energy_residual_ratio": 0.010620163614451995,
    "measurement_decimation": 80,
    "metrics": {
      "contact_state_time_s": {
        "two_wheels": 0.10000000000000006
      },
      "duration_s": 0.10000000000000006,
      "flight_time_s": 0.0,
      "front_lift_time_s": 0.0,
      "front_load_fraction_mean": 0.4084571520162067,
      "front_load_fraction_min": 0.3898128106450679,
      "front_unloaded_time_s": 0.0,
      "max_front_clearance_m": 0.0,
      "max_pitch_rate_up_rad_s": 0.06602960989192447,
      "max_relative_pitch_rad": 0.0012749698978237536,
      "min_front_load_n": 392.54880126667035,
      "rear_slip_distance_m": 0.0009360237072205245,
      "wheelie_candidate_time_s": 0.0,
      "wheelie_episode_records": [],
      "wheelie_episodes": 0,
      "wheelie_time_s": 0.0
    },
    "model_status": {
      "calibration_status": "parameterized_unvalidated",
      "first_model_violation": {
        "interval_id": 3,
        "reasons": [
          "energy.constraint_work"
        ],
        "time_s": 0.00375
      },
      "model_valid": false,
      "model_violation_counts": {
        "energy.constraint_work": 20,
        "foot_rear.normal": 2
      },
      "numerically_valid": true
    },
    "numerically_valid": true,
    "parameters_validated_against_measurements": false,
    "physical_downsampling": false,
    "policy_inputs": "proper acceleration, pitch gyro, wheel/crank encoders, motor/human torque sensors",
    "privileged_outputs": "contact loads/clearances, road-relative pitch, CoM and simulator speed",
    "run_metadata": {},
    "sensor_config": {
      "acceleration_bias_mps2": [
        0.0,
        0.0,
        0.0
      ],
      "acceleration_std_mps2": 0.05,
      "dropout_probability": 0.0,
      "encoder_std_rad_s": 0.01,
      "gyro_bias_rad_s": 0.0,
      "gyro_std_rad_s": 0.002,
      "imu_enabled": true,
      "latency_s": 0.01,
      "maximum_age_s": 0.1,
      "sample_period_s": 0.005,
      "torque_std_nm": 0.05
    },
    "source_changed_during_run": false,
    "terminated": false,
    "timestep_alignment": "truth/sensor source is the last incoming physical state, not the endpoint",
    "truncated": true,
    "valid_for_learning": false
  },
  "resolved_config": {
    "active_suspension": {
      "coil": {
        "bumper_length_mm": 10.0,
        "bumper_peak_n": 7000.0,
        "preload_mm": 0.0,
        "rate_n_m": 114600.0,
        "stroke_mm": 65.0
      },
      "fork_damper": {
        "c_hsc": 333.21257864384114,
        "c_lsc": 634.198904787903,
        "c_reb": 909.414284430181,
        "frac_hsc": 0.5,
        "frac_lsc": 0.5,
        "frac_reb": 0.5294117647058824
      },
      "fork_pressure_psi": 85.2,
      "fork_tokens": 2,
      "shock_damper": {
        "c_hbo": 9685.766774493648,
        "c_hsc": 900.6252313054151,
        "c_lsc": 1596.0084671604288,
        "c_reb": 3990.454499701241,
        "frac_hbo": 0.5,
        "frac_hsc": 0.5,
        "frac_lsc": 0.5,
        "frac_reb": 0.5,
        "lockout_firm": false
      }
    },
    "allocation_numerical_guards": {
      "grip_radius_fraction": 1e-06,
      "support_force_n": 1e-05,
      "support_moment_nm": 1e-06
    },
    "compiled_contact_solver": {
      "front": {
        "condim": 3,
        "friction": [
          1.1,
          0.005,
          0.0001
        ],
        "native_enabled": false,
        "solimp": [
          0.9,
          0.95,
          0.001,
          0.5,
          2.0
        ],
        "solref": [
          -130000.0,
          -800.0
        ],
        "units": "MuJoCo solver parameters; not tire material stiffness or damping"
      },
      "rear": {
        "condim": 3,
        "friction": [
          1.1,
          0.005,
          0.0001
        ],
        "native_enabled": false,
        "solimp": [
          0.9,
          0.95,
          0.001,
          0.5,
          2.0
        ],
        "solref": [
          -130000.0,
          -800.0
        ],
        "units": "MuJoCo solver parameters; not tire material stiffness or damping"
      }
    },
    "compiled_equality_solimp": [
      [
        0.99,
        0.999,
        0.0001,
        0.5,
        2.0
      ],
      [
        0.99,
        0.999,
        0.0001,
        0.5,
        2.0
      ],
      [
        0.9,
        0.95,
        0.001,
        0.5,
        2.0
      ],
      [
        0.9,
        0.95,
        0.001,
        0.5,
        2.0
      ],
      [
        0.9,
        0.95,
        0.001,
        0.5,
        2.0
      ],
      [
        0.9,
        0.95,
        0.001,
        0.5,
        2.0
      ],
      [
        0.9,
        0.95,
        0.001,
        0.5,
        2.0
      ]
    ],
    "compiled_equality_solref": [
      [
        0.0025,
        1.0
      ],
      [
        0.0025,
        1.0
      ],
      [
        0.0025,
        1.0
      ],
      [
        0.0025,
        1.0
      ],
      [
        0.0025,
        1.0
      ],
      [
        0.0025,
        1.0
      ],
      [
        0.0025,
        1.0
      ]
    ],
    "geometry_and_suspension": {
      "bb_drop": 22.5,
      "crank_length": 165.0,
      "effective_seat_angle_deg": 77.0,
      "fork_air_tokens": 2,
      "fork_damping": 450.0,
      "fork_hsc": 2,
      "fork_initial_psi": 85.2,
      "fork_lsc": 7,
      "fork_offset": 44.0,
      "fork_rebound": 9,
      "fork_stiffness": 6000.0,
      "fork_travel": 180.0,
      "front_wheel_radius": 372.0,
      "head_angle_deg": 64.0,
      "headtube_length": 120.0,
      "reach": 480.0,
      "rear_wheel_radius": 352.0,
      "rear_wheel_travel": 180.0,
      "shock_damping": 1200.0,
      "shock_eye_to_eye": 205.0,
      "shock_hbo": 2,
      "shock_hsc": 2,
      "shock_lockout": false,
      "shock_lsc": 7,
      "shock_rebound": 7,
      "shock_stiffness": 114600.0,
      "shock_stroke": 65.0,
      "stack": 646.0,
      "wheel_radius": 372.0,
      "wheelbase": 1280.55
    },
    "joint_envelope_sha256": "4e9cacbce5b7a39d369f3e1d7eacedbef5386c56e1707045ea140621baa43f64",
    "joint_envelope_status": "declared_unvalidated",
    "joint_strength_sha256": "0921312611e18611da3e384d2827e63c183eb642f0b9be1cbbf0a58480eee5f8",
    "joint_strength_status": "declared_unvalidated",
    "mass_budget": {
      "battery_mass": 4.3,
      "chainstay_mass": 1.25,
      "crank_pedals_mass": 0.85,
      "fork_lowers_mass": 1.5,
      "frame_structure_mass": 3.2,
      "front_wheel_mass": 2.4,
      "motor_mass": 2.9,
      "rear_wheel_mass": 2.8,
      "rocker_mass": 0.4,
      "saddle_post_mass": 0.9,
      "seatstay_mass": 1.05,
      "shock_damper_mass": 0.6,
      "shock_yoke_mass": 0.3,
      "stanchions_mass": 0.85,
      "steer_assembly_mass": 1.1,
      "wheel_hub_core_fraction": 0.25,
      "wheel_rim_tire_fraction": 0.75
    },
    "pedal_support_half_length_m": {
      "front": 0.05,
      "rear": 0.05
    },
    "physics": {
      "articulated": {
        "activation_tau_s": 0.0,
        "active_positive_power_limit_w": 450.0,
        "arm_reach_fraction": 0.92,
        "balance_dwell_s": 0.5,
        "balance_floor_kmh": 4.0,
        "balance_grace_s": 3.0,
        "bar_support_fraction": 0.12,
        "coasting_brake_d_nm_s_rad": 2.0,
        "coasting_brake_limit_nm": 60.0,
        "coupled_task_control": false,
        "foot_mu": 0.9,
        "grip_attachment": "connect",
        "grip_c_ns_m": 150.0,
        "grip_capture_distance_m": 0.02,
        "grip_capture_speed_mps": 0.2,
        "grip_k_n_m": 4000.0,
        "grip_pair_force_limit_n": null,
        "grip_pull_per_hand_n": 300.0,
        "grip_release_distance_m": 0.12,
        "joint_envelope_path": "/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp/tests/fixtures/migration/config/rider_joint_envelope_anatomical.json",
        "joint_envelope_soft_k_nm_rad": 100.0,
        "joint_envelope_soft_margin_rad": 0.1,
        "joint_kd_nms_rad": 15.0,
        "joint_kp_nm_rad": 600.0,
        "joint_limit_nm": 100.0,
        "joint_passive_damping_nms_rad": 0.0,
        "joint_power_limit_w": 250.0,
        "joint_speed_limit_rad_s": 20.0,
        "joint_strength_path": "/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp/tests/fixtures/migration/config/rider_strength_reference.json",
        "link_max_gap_m": 0.005,
        "pedal_ankle_amplitude_rad": 0.1,
        "pedal_attachment": "spindle",
        "pedal_c_ns_m": 100.0,
        "pedal_min_normal_n": 20.0,
        "pedal_patch_half_length_m": 0.025,
        "pedal_scrape_fraction": 0.5,
        "pedal_support_fraction": 0.33,
        "pedal_torque_ripple": 0.5,
        "posture_pitch_d_nms_rad": 60.0,
        "posture_pitch_k_nm_rad": 600.0,
        "posture_pitch_limit_nm": 60.0,
        "posture_sole_depth_m": 0.003,
        "posture_translation_d_ns_m": 150.0,
        "posture_translation_k_n_m": 1500.0,
        "posture_translation_limit_n": 400.0,
        "return_foot_preload_n": 40.0,
        "road_lookahead_m": 2.0,
        "saddle_attachment": "pin",
        "saddle_mu": 0.6,
        "saddle_patch_half_length_m": 0.045,
        "saddle_reserve_weight_fraction": 0.15,
        "stance_blend_load_n": 50.0,
        "support_c_ns_m": 500.0,
        "support_k_n_m": 100000.0,
        "support_length_m": 0.1,
        "support_mu": 0.8,
        "support_pad_radius_m": 0.02,
        "support_tangent_k_n_m": 20000.0,
        "swing_clearance_m": 0.003
      },
      "closure_time_constant_s": 0.0025,
      "control_period_s": 0.005,
      "drive": {
        "assist": {
          "cutoff_mps": 6.944444444444445,
          "engage_torque_nm": 4.0,
          "gain": 2.0,
          "gate_min_crank_rad_s": 0.08726646259971647,
          "max_power": 500.0,
          "max_torque": 80.0,
          "mode": "turbo",
          "profile": "bosch_cx_gen4",
          "slew": 400.0,
          "taper_width_mps": 0.5555555555555556,
          "tau": 0.05,
          "torque_curve": [
            [
              0.0,
              85.0
            ],
            [
              120.0,
              85.0
            ],
            [
              120.1,
              47.7
            ],
            [
              180.0,
              0.0
            ]
          ]
        },
        "battery": {
          "copper_w_per_nm2": 0.02,
          "enabled": false,
          "energy_j": 1800000.0,
          "idle_w": 5.0,
          "speed_w_per_rad_s2": 0.0
        },
        "bearing_c_nms_rad": 0.03,
        "brake_ceiling_nm": 200.0,
        "chain_c_ns_m": 10.0,
        "chain_k_n_m": 200000.0,
        "crank_phase_rad": 0.0,
        "freehub_c_nms_rad": 0.5,
        "freehub_k_nm_rad": 1000.0,
        "gearing": {
          "chain_pitch_m": 0.0127,
          "front_teeth": 22,
          "rear_teeth": 51
        },
        "human_torque_nm": 0.0,
        "motor_clutch": false,
        "pedaling": {
          "coast_above_rpm": 120.0,
          "coast_cadence_tau_s": 0.35,
          "effort_slew_nm_s": 300.0,
          "enabled": true,
          "mash_cadence_rpm": 45.0,
          "mash_torque_nm": 60.0,
          "resume_below_rpm": 105.0,
          "rollback_brake": true,
          "rollback_demand": 1.0,
          "rollback_engage_mps": 0.25,
          "rollback_release_mps": 0.0,
          "stop_time_s": 0.35
        },
        "rotor_inertia_kgm2": 0.0,
        "shifting": {
          "cadence_smoothing_tau_s": 0.35,
          "cassette": [
            10,
            12,
            14,
            16,
            18,
            21,
            24,
            28,
            33,
            39,
            45,
            51
          ],
          "enabled": true,
          "shift_cooldown_s": 0.4,
          "shift_cut_duration_s": 0.2,
          "target_cadence_max_rpm": 110.0,
          "target_cadence_min_rpm": 75.0,
          "torque_factor": 0.3,
          "upshift_slip_limit_mps": 0.5,
          "upshift_slip_mode": "legacy_signed"
        },
        "torque_ripple": 0.35,
        "transmission_model": "ideal_mid_drive"
      },
      "drive_mode": "articulated_effort",
      "end_stops": {
        "damping_n_s_m": 500.0,
        "overtravel_m": 0.01,
        "provenance": "synthetic",
        "reference_deflection_m": 0.01,
        "reference_force_n": 7000.0
      },
      "equilibrium_cache_enabled": true,
      "equilibrium_refine_after_s": 1.0,
      "equilibrium_refine_period_s": 3.0,
      "equilibrium_state_file": null,
      "initial_front_brake": 1.0,
      "initial_rear_brake": 1.0,
      "initial_speed_mps": 0.0,
      "physics_mode": "physical",
      "pitch_assist": false,
      "resistance": {
        "cda_m2": 0.5,
        "crr": 0.015,
        "point_body_m": [
          0.0,
          0.0,
          0.5
        ],
        "rho_kg_m3": 1.2,
        "rolling_taper_rad_s": 0.2,
        "wind_world_mps": [
          0.0,
          0.0,
          0.0
        ]
      },
      "seated_climb": {
        "enabled": true,
        "front_load_share_target": 0.3,
        "lean_gain": 1.0,
        "lean_rate_rad_s": 0.5,
        "lean_trim_gain_rad_s": 0.3,
        "lean_trim_limit_rad": 0.15,
        "max_backward_lean_rad": 0.1,
        "max_crank_torque_nm": 60.0,
        "max_forward_lean_rad": 0.35,
        "orientation_tau_s": 0.5,
        "period_s": 0.01,
        "reaction_delay_s": 0.15,
        "surge_budget_s": 15.0,
        "surge_grade": 0.2,
        "surge_power_w": 400.0,
        "surge_recovery_rate": 0.3333333333333333,
        "target_crank_power_w": 250.0,
        "torque_slew_nm_s": 300.0,
        "trim_dead_time_s": 0.2
      },
      "timestep_s": 0.00125,
      "tires": {
        "backend": "compliant_2d",
        "distinct_normal_deg": 20.0,
        "distributed": {
          "calibration_status": "experimental_unvalidated",
          "density": {
            "damping_ns_m": 2000.0,
            "knots_m": [
              0.0,
              0.004,
              0.012
            ],
            "provenance": "synthetic density; not fitted or measured",
            "stiffness_n_m": [
              300000.0,
              0.0,
              0.0
            ]
          },
          "fitting_dataset_id": "none-synthetic",
          "front_material": null,
          "rear_material": null,
          "station_count": 128,
          "valid_deflection_range_m": [
            0.0,
            0.04
          ],
          "valid_load_range_n": [
            0.0,
            2000.0
          ]
        },
        "front": {
          "material": {
            "pressure_pa_gauge": 200000.0,
            "provenance": "synthetic",
            "radial_c_ns_m": 800.0,
            "radial_k_n_m": 130000.0,
            "valid_load_range_n": [
              0.0,
              2000.0
            ]
          },
          "mu": 1.1,
          "relaxation_length_m": 0.2,
          "tangent_k_n_m": 20000.0
        },
        "pneumatic": {
          "front": {
            "area_factor": 0.4435,
            "carcass_stiffness_n_mm2": 0.1194,
            "casing_width_mm": 60.0,
            "compressed_casing_mm": 6.0,
            "contact_length_factor": 0.8589,
            "crr_reference": 0.011,
            "hysteresis_rate_eps_mps": 0.01,
            "loss_factor": 0.0662,
            "name": "Schwalbe Magic Mary 29x2.4",
            "outer_radius_mm": 372.0,
            "pressure_bar": 1.5,
            "rate_relaxation_s": 0.2,
            "rate_stiffening": 0.2484,
            "relaxation_length_mm": 60.0,
            "rim_radius_mm": 320.0,
            "rim_stiffness_n_mm2": 28.0,
            "section_height_mm": 57.0,
            "tread_loss_crr": 0.00586
          },
          "rear": {
            "area_factor": 0.4576,
            "carcass_stiffness_n_mm2": 0.1225,
            "casing_width_mm": 60.0,
            "compressed_casing_mm": 6.0,
            "contact_length_factor": 0.8801,
            "crr_reference": 0.0103,
            "hysteresis_rate_eps_mps": 0.01,
            "loss_factor": 0.069,
            "name": "Schwalbe Hans Dampf 27.5x2.4",
            "outer_radius_mm": 352.0,
            "pressure_bar": 1.7,
            "rate_relaxation_s": 0.2,
            "rate_stiffening": 0.2498,
            "relaxation_length_mm": 60.0,
            "rim_radius_mm": 300.0,
            "rim_stiffness_n_mm2": 28.0,
            "section_height_mm": 55.0,
            "tread_loss_crr": 0.00481
          },
          "surface": null,
          "tier": "fast"
        },
        "rear": {
          "material": {
            "pressure_pa_gauge": 200000.0,
            "provenance": "synthetic",
            "radial_c_ns_m": 800.0,
            "radial_k_n_m": 130000.0,
            "valid_load_range_n": [
              0.0,
              2000.0
            ]
          },
          "mu": 1.1,
          "relaxation_length_m": 0.2,
          "tangent_k_n_m": 20000.0
        },
        "significance_fraction": 0.05,
        "significant_delta_m": 0.0001,
        "surface_mode": "track"
      }
    },
    "rider": {
      "arm_resonance_hz": 4.0,
      "bar_share": 0.12,
      "damping_ratio": 0.4,
      "height_m": 1.8,
      "helmet_mass_kg": 0.4,
      "inseam_m": null,
      "leg_resonance_hz": 5.0,
      "legs": "rigid",
      "mass_kg": 80.0,
      "pedal_share": 0.33,
      "saddle_interface_c_ns_m": 2762.0,
      "saddle_interface_k_n_m": 101000.0,
      "saddle_share": 0.55,
      "torso_resonance_hz": 7.0,
      "variant": "articulated_planar"
    },
    "seed": 210,
    "target_speed_kmh": null,
    "terrain_sha256": "d9326d33e13d7c2574a8995baa3408dd625ee378209e0de4f8397db1dbe16928",
    "track": {
      "description": "No obstacles; sag, rolling resistance and coast-down reference.",
      "length_m": 112.0,
      "name": "flat",
      "obstacles": [],
      "surface": "hardpack"
    }
  }
}
```

## Field and coordinate coverage

The complete finite field inventory is `field_catalog`; model-address identities are `coordinate_catalog`. `_present` columns describe existence independently of nullable values. Parent `_valid` flags distinguish an explicitly null parent from an empty or absent parent. Structured children carry their own optionality flags.

Diagnostics not represented by this catalog were not recorded and cannot be reconstructed as source facts. Observation delivery and acquisition timestamps are separate; transitions reference observations rather than duplicating them.
