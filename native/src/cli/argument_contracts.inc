// Static source/captured-parser contracts plus documented native-only flags;
// no Python build generator. The frozen Python migration contract is separate.
static constexpr std::string_view kArgumentContracts = R"CLICONTRACT({
  "bike-sim": {
    "options": [
      {
        "names": [
          "--print-table"
        ],
        "dest": "print_table",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--plot"
        ],
        "dest": "plot",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--plot-damper"
        ],
        "dest": "plot_damper",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--export-json"
        ],
        "dest": "export_json",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--export-mujoco"
        ],
        "dest": "export_mujoco",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--playground"
        ],
        "dest": "playground",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--output-plots-dir"
        ],
        "dest": "output_plots_dir",
        "default": "output/plots",
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--output-models-dir"
        ],
        "dest": "output_models_dir",
        "default": "output/models",
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--all"
        ],
        "dest": "all",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      }
    ],
    "help": ""
  },
  "bike-playground": {
    "options": [],
    "help": ""
  },
  "bike-export": {
    "options": [
      {
        "names": [
          "--output-dir",
          "-o"
        ],
        "dest": "output_dir",
        "default": "output/models",
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--json-only"
        ],
        "dest": "json_only",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--xml-only"
        ],
        "dest": "xml_only",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      }
    ],
    "help": ""
  },
  "bike-ride": {
    "options": [
      {"names":["--spatial"],"dest":"spatial","default":null,"type":"str","choices":["planar","3d"],"flag":false,"required":false,"positional":false},
      {"names":["--terrain3d"],"dest":"terrain3d","default":null,"type":"str","choices":null,"flag":false,"required":false,"positional":false},
      {"names":["--threads"],"dest":"threads","default":1,"type":"int","choices":null,"flag":false,"required":false,"positional":false},
      {
        "names": [
          "--track"
        ],
        "dest": "track",
        "default": "road_worn",
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--seed"
        ],
        "dest": "seed",
        "default": null,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--length"
        ],
        "dest": "length",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--speed"
        ],
        "dest": "speed",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--headless"
        ],
        "dest": "headless",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--sag"
        ],
        "dest": "sag",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--out"
        ],
        "dest": "out",
        "default": "output/ride",
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--decimate"
        ],
        "dest": "decimate",
        "default": null,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider"
        ],
        "dest": "rider",
        "default": null,
        "type": "str",
        "choices": [
          "none",
          "lumped",
          "seated",
          "articulated_planar"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-mass"
        ],
        "dest": "rider_mass",
        "default": 80.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-height"
        ],
        "dest": "rider_height",
        "default": 1.8,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-inseam"
        ],
        "dest": "rider_inseam",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--no-rider"
        ],
        "dest": "no_rider",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--legs"
        ],
        "dest": "legs",
        "default": null,
        "type": "str",
        "choices": [
          "articulated",
          "rigid"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--preview"
        ],
        "dest": "preview",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--dump-track"
        ],
        "dest": "dump_track",
        "default": null,
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--list-tracks"
        ],
        "dest": "list_tracks",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--no-plots"
        ],
        "dest": "no_plots",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--html"
        ],
        "dest": "html",
        "default": null,
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--tyre-model"
        ],
        "dest": "tyre_model",
        "default": null,
        "type": "str",
        "choices": [
          "sphere",
          "pneumatic"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--tyre-tier"
        ],
        "dest": "tyre_tier",
        "default": null,
        "type": "str",
        "choices": [
          "fast",
          "detailed"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--tyre-pressure"
        ],
        "dest": "tyre_pressure",
        "default": null,
        "type": "_parse_tyre_pressures",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--drive-mode"
        ],
        "dest": "drive_mode",
        "default": null,
        "type": "str",
        "choices": [
          "motor",
          "pedal",
          "pedelec"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--assist"
        ],
        "dest": "assist",
        "default": null,
        "type": "str",
        "choices": [
          "off",
          "eco",
          "tour",
          "sport",
          "turbo",
          "emtb"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--gearing"
        ],
        "dest": "gearing",
        "default": null,
        "type": "_parse_gearing",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--ripple-depth"
        ],
        "dest": "ripple_depth",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--crank-phase"
        ],
        "dest": "crank_phase",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--visual-pedalling"
        ],
        "dest": "visual_pedalling",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--surface"
        ],
        "dest": "surface",
        "default": null,
        "type": "str",
        "choices": [
          "asphalt",
          "hardpack",
          "loose",
          "wet",
          "scree"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--physics"
        ],
        "dest": "physics",
        "default": null,
        "type": "str",
        "choices": [
          "legacy",
          "physical"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--backend"
        ],
        "dest": "backend",
        "default": "native",
        "type": "str",
        "choices": [
          "native"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--close-on-finish"
        ],
        "dest": "close_on_finish",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--time-scale"
        ],
        "dest": "time_scale",
        "default": null,
        "type": "int",
        "choices": [
          1,
          2,
          4,
          8
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--record"
        ],
        "dest": "record",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--check-policy"
        ],
        "dest": "check_policy",
        "default": null,
        "type": "str",
        "choices": [
          "strict",
          "diagnostic"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--drive"
        ],
        "dest": "drive",
        "default": null,
        "type": "str",
        "choices": [
          "coast",
          "ideal_speed_control",
          "crank_effort",
          "articulated_effort"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--physics-config"
        ],
        "dest": "physics_config",
        "default": null,
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--initial-speed"
        ],
        "dest": "initial_speed",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--human-torque"
        ],
        "dest": "human_torque",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--assist-gain"
        ],
        "dest": "assist_gain",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--timestep"
        ],
        "dest": "timestep",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--duration"
        ],
        "dest": "duration",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--research"
        ],
        "dest": "research",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--async-telemetry"
        ],
        "dest": "async_telemetry",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--policy"
        ],
        "dest": "policy",
        "default": null,
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--control-period"
        ],
        "dest": "control_period",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--sensor-period"
        ],
        "dest": "sensor_period",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--sensor-delay"
        ],
        "dest": "sensor_delay",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--actuator-delay"
        ],
        "dest": "actuator_delay",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--road-resolution"
        ],
        "dest": "road_resolution",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--demand"
        ],
        "dest": "demand",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-program"
        ],
        "dest": "rider_program",
        "default": null,
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      }
    ],
    "help": "usage: bike-ride [-h] [options]\n\nNative bicycle simulator.\n  --close-on-finish  Close an interactive physical/research viewer when the run finishes (requires --duration).\n"
  },
  "bike-settle": {
    "options": [
      {"names":["--spatial"],"dest":"spatial","default":null,"type":"str","choices":["planar","3d"],"flag":false,"required":false,"positional":false},
      {"names":["--terrain3d"],"dest":"terrain3d","default":null,"type":"str","choices":null,"flag":false,"required":false,"positional":false},
      {"names":["--threads"],"dest":"threads","default":1,"type":"int","choices":null,"flag":false,"required":false,"positional":false},
      {
        "names": [
          "--physics-config"
        ],
        "dest": "physics_config",
        "default": null,
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--physics"
        ],
        "dest": "physics",
        "default": null,
        "type": "str",
        "choices": [
          "legacy",
          "physical"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--track"
        ],
        "dest": "track",
        "default": "road_worn",
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--seed"
        ],
        "dest": "seed",
        "default": null,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--length"
        ],
        "dest": "length",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider"
        ],
        "dest": "rider",
        "default": null,
        "type": "str",
        "choices": [
          "none",
          "lumped",
          "seated",
          "articulated_planar"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-mass"
        ],
        "dest": "rider_mass",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-height"
        ],
        "dest": "rider_height",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-inseam"
        ],
        "dest": "rider_inseam",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--no-rider"
        ],
        "dest": "no_rider",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--speed"
        ],
        "dest": "speed",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--drive"
        ],
        "dest": "drive",
        "default": null,
        "type": "str",
        "choices": [
          "coast",
          "ideal_speed_control",
          "crank_effort",
          "articulated_effort"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--timestep"
        ],
        "dest": "timestep",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--initial-speed"
        ],
        "dest": "initial_speed",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--human-torque"
        ],
        "dest": "human_torque",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--assist-gain"
        ],
        "dest": "assist_gain",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--assist"
        ],
        "dest": "assist",
        "default": null,
        "type": "str",
        "choices": [
          "off",
          "eco",
          "tour",
          "sport",
          "turbo",
          "emtb"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--crank-phase"
        ],
        "dest": "crank_phase",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--max-steps"
        ],
        "dest": "max_steps",
        "default": 400000,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--tolerance"
        ],
        "dest": "tolerance",
        "default": 0.05,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--out"
        ],
        "dest": "out",
        "default": null,
        "type": "str",
        "choices": null,
        "flag": false,
        "required": true,
        "positional": false
      }
    ],
    "help": ""
  },
  "bike-research": {
    "options": [
      {"names":["--spatial"],"dest":"spatial","default":null,"type":"str","choices":["planar","3d"],"flag":false,"required":false,"positional":false},
      {"names":["--terrain3d"],"dest":"terrain3d","default":null,"type":"str","choices":null,"flag":false,"required":false,"positional":false},
      {"names":["--threads"],"dest":"threads","default":1,"type":"int","choices":null,"flag":false,"required":false,"positional":false},
      {
        "names": [
          "--backend"
        ],
        "dest": "backend",
        "default": "native",
        "type": "str",
        "choices": [
          "native"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--physics-config"
        ],
        "dest": "physics_config",
        "default": null,
        "type": "Path",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--road-resolution"
        ],
        "dest": "road_resolution",
        "default": 0.005,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--sensor-period"
        ],
        "dest": "sensor_period",
        "default": 0.005,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-program"
        ],
        "dest": "rider_program",
        "default": null,
        "type": "Path",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--policy"
        ],
        "dest": "policy",
        "default": null,
        "type": "str",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--record"
        ],
        "dest": "record",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--check-policy"
        ],
        "dest": "check_policy",
        "default": null,
        "type": "str",
        "choices": [
          "strict",
          "diagnostic"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--diagnostic-model-limits"
        ],
        "dest": "diagnostic_model_limits",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--scenario"
        ],
        "dest": "scenario",
        "default": "rough_uphill",
        "type": "str",
        "choices": [
          "flat",
          "uphill",
          "rough_uphill",
          "crest",
          "low_grip",
          "step_up",
          "generated",
          "alpine"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--gen-spec"
        ],
        "dest": "gen_spec",
        "default": null,
        "type": "Path",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--track-file"
        ],
        "dest": "track_file",
        "default": null,
        "type": "Path",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--seed"
        ],
        "dest": "seed",
        "default": 0,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider"
        ],
        "dest": "rider",
        "default": "articulated_planar",
        "type": "str",
        "choices": [
          "lumped",
          "articulated_planar"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-mass"
        ],
        "dest": "rider_mass",
        "default": 80.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-height"
        ],
        "dest": "rider_height",
        "default": 1.8,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-random"
        ],
        "dest": "rider_random",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rider-seed"
        ],
        "dest": "rider_seed",
        "default": null,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--posture"
        ],
        "dest": "posture",
        "default": "neutral",
        "type": "str",
        "choices": [
          "neutral",
          "forward",
          "crouched",
          "standing"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--duration"
        ],
        "dest": "duration",
        "default": 3.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--dt"
        ],
        "dest": "dt",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--control-period"
        ],
        "dest": "control_period",
        "default": 0.01,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--actuator-delay"
        ],
        "dest": "actuator_delay",
        "default": 0.005,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--sensor-delay"
        ],
        "dest": "sensor_delay",
        "default": 0.01,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--ideal-sensors"
        ],
        "dest": "ideal_sensors",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--initial-brake-demand"
        ],
        "dest": "initial_brake_demand",
        "default": 1.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--initial-speed"
        ],
        "dest": "initial_speed",
        "default": 2.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--human-torque"
        ],
        "dest": "human_torque",
        "default": 0.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--motor-torque"
        ],
        "dest": "motor_torque",
        "default": 80.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--demand"
        ],
        "dest": "demand",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--demand-file"
        ],
        "dest": "demand_file",
        "default": null,
        "type": "Path",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--assist"
        ],
        "dest": "assist",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--motor-limit"
        ],
        "dest": "motor_limit",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--motor-max-torque"
        ],
        "dest": "motor_max_torque",
        "default": 80.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--motor-max-power"
        ],
        "dest": "motor_max_power",
        "default": 500.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--transmission"
        ],
        "dest": "transmission",
        "default": "ideal_mid_drive",
        "type": "str",
        "choices": [
          "elastic_chain",
          "ideal_mid_drive",
          "geometric_ideal_mid_drive"
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--chain-stiffness"
        ],
        "dest": "chain_stiffness",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--freehub-stiffness"
        ],
        "dest": "freehub_stiffness",
        "default": null,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--front-teeth"
        ],
        "dest": "front_teeth",
        "default": 34,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--rear-teeth"
        ],
        "dest": "rear_teeth",
        "default": 51,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--fast-telemetry"
        ],
        "dest": "fast_telemetry",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--record-decimation"
        ],
        "dest": "record_decimation",
        "default": 80,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--energy-tolerance"
        ],
        "dest": "energy_tolerance",
        "default": 0.05,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--out"
        ],
        "dest": "out",
        "default": "output/antiwheelie",
        "type": "Path",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--overwrite"
        ],
        "dest": "overwrite",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      }
    ],
    "help": ""
  },
  "bike-batch": {
    "options": [
      {
        "names": [
          "--spec"
        ],
        "dest": "spec",
        "default": null,
        "type": "Path",
        "choices": null,
        "flag": false,
        "required": true,
        "positional": false
      },
      {
        "names": [
          "--out"
        ],
        "dest": "out",
        "default": null,
        "type": "Path",
        "choices": null,
        "flag": false,
        "required": true,
        "positional": false
      },
      {
        "names": [
          "--jobs",
          "-j"
        ],
        "dest": "jobs",
        "default": null,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--serial"
        ],
        "dest": "serial",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--dry-run"
        ],
        "dest": "dry_run",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--overwrite"
        ],
        "dest": "overwrite",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      }
    ],
    "help": ""
  },
  "bike-replay": {
    "options": [
      {"names":["--checked"],"dest":"checked","default":false,"type":"str","choices":null,"flag":true,"required":false,"positional":false},
      {
        "names": [
          "recording"
        ],
        "dest": "recording",
        "default": null,
        "type": "Path",
        "choices": null,
        "flag": false,
        "required": true,
        "positional": true
      },
      {
        "names": [
          "--episode"
        ],
        "dest": "episode",
        "default": null,
        "type": "int",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--viewer"
        ],
        "dest": "viewer",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--time-scale"
        ],
        "dest": "time_scale",
        "default": null,
        "type": "int",
        "choices": [
          1,
          2,
          4,
          8
        ],
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--view"
        ],
        "dest": "view",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--speed"
        ],
        "dest": "speed",
        "default": 1.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--loop"
        ],
        "dest": "loop",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--fps"
        ],
        "dest": "fps",
        "default": 60.0,
        "type": "float",
        "choices": null,
        "flag": false,
        "required": false,
        "positional": false
      },
      {
        "names": [
          "--dry-run"
        ],
        "dest": "dry_run",
        "default": false,
        "type": "str",
        "choices": null,
        "flag": true,
        "required": false,
        "positional": false
      }
    ],
    "help": ""
  }
})CLICONTRACT";
