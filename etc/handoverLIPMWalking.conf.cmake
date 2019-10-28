{
	"initial_plan": "ashibumi",

	"mpc":
	{
		"weights":
		{
			"jerk": 1.0,
			"vel": [10.0, 100.0],
			"zmp": 1000.0
		}
	},


	"RunObservers": ["Encoder"],
	"UpdateObservers": ["Encoder"].

  "stabilizer":
  {
    "fdqp_weights":
    {
      "net_wrench": 10000.0,
      "ankle_torque": 100.0,
      "pressure": 1.0
    },

    "tasks":
    {
      "com":
      {
        "stiffness": [1000.0, 1000.0, 100.0],
        "weight": 1000.0
      },
      "contact":
      {
        "damping": 300.0,
        "stiffness": 1.0,
        "weight": 10000.0
      },
      "swing_foot":
      {
        "stiffness": 2000.0,
        "weight": 500.0
      }
    },

    "vdc":
    {
      "frequency": 1.0,
      "stiffness": 1000.0
    },

    "zmpcc":
    {
      "integrator_leak_rate": 0.1,
    }
  },



  "tasks":
  {
    "pelvis":
    {
      "stiffness": 10.0,
      "weight": 100.0
    },
    "posture":
    {
      "stiffness": 1.0,
      "weight": 10.0
    },
    "torso":
    {
      "pitch": 0.1,
      "stiffness": 10.0,
      "weight": 100.0
    }
  },


  "robot_models":
  {
    "hrp2_drc": // robot-specific settings for HRP-2Kai
  	{
  		"admittance":
  		{
  			"com": [0.0, 0.0],
  			"cop": [0.01, 0.01],
  			"dfz": 0.0001,
        "dfz_damping": 0.0
  		},

  		"com":
  		{
  			"active_joints":
        [
  				"Root",
  				"RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2", "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
  				"LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2", "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5"
  			],
  			"height": 0.87,
  			"max_height": 0.92,
  			"min_height": 0.6
  		},

      "dcm_tracking":
      {
        "gains":
        {
          "prop": 4.0,
          "integral": 15.0,
          "deriv": 0.5
        },
        "derivator_time_constant": 1.0,
        "integrator_time_constant": 15.0
      },

      "sole":
  		{
  			"half_length": 0.108,
  			"half_width": 0.07,
  			"friction": 0.7
  		},

      "step_width": 0.2,
  		"torso": "CHEST_LINK1"
  	}
  },



	"plans":
	{
    "hrp2_drc":
    {
  		"ashibumi": // stepping in place
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.04,
  			"contacts":
  			[
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
  			],
  		},

  		"ashibumi_fast":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" }
  			]
  		},


  		"HANDOVER_fwd_20cm_10cm":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },

  				{ "pose": { "translation": [0.2,  -0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.2,   0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "LeftFootCenter" },

  				{ "pose": { "translation": [0.3,  -0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.3,   0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "LeftFootCenter" },
  			]
  		},

  		"HANDOVER_back_20cm_10cm":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },

  				{ "pose": { "translation": [-0.1,  -0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.1,   0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "LeftFootCenter"  },

  				{ "pose": { "translation": [-0.3, -0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.3,   0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "LeftFootCenter"  },
  			]
  		},


  		"HANDOVER_fwd_20cm_20cm":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },

  				{ "pose": { "translation": [0.2,  -0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.2,   0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "LeftFootCenter" },

  				{ "pose": { "translation": [0.4,  -0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.4,   0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "LeftFootCenter" },
  			]
  		},

  		"HANDOVER_back_20cm_20cm":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },

  				{ "pose": { "translation": [-0.2,  -0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.2,   0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "LeftFootCenter"  },

  				{ "pose": { "translation": [-0.4, -0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.4,   0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "LeftFootCenter"  },
  			]
  		},


  		"HANDOVER_1stepCycle_fwd_10cm":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.1,  -0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.1,   0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "LeftFootCenter" },
  			]
  		},

  		"HANDOVER_1stepCycle_back_10cm":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.1, -0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.1,   0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "LeftFootCenter"  },
  			]
  		},


  		"HANDOVER_1stepCycle_fwd_15cm":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.15,  -0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.15,   0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "LeftFootCenter" },
  			]
  		},

  		"HANDOVER_1stepCycle_back_15cm":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.15, -0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.15,   0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "LeftFootCenter"  },
  			]
  		},


  		"HANDOVER_1stepCycle_fwd_20cm":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.2,  -0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.2,   0.09, 0.0] }, "ref_vel": [0.10, 0.0, 0.0], "surface": "LeftFootCenter" },
  			]
  		},

  		"HANDOVER_1stepCycle_back_20cm":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.2, -0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.2,   0.09, 0.0] }, "ref_vel": [-0.10, 0.0, 0.0], "surface": "LeftFootCenter"  },
  			]
  		},


  		"HANDOVER_1stepCycle_fwd_30cm":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.3,  -0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.3,   0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
  			]
  		},

  		"HANDOVER_1stepCycle_back_30cm":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.3, -0.09, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.3,   0.09, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "LeftFootCenter"  },
  			]
  		},


  		"HANDOVER_1stepCycle_fwd_40cm":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.4,  -0.09, 0.0] }, "ref_vel": [0.20, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.4,   0.09, 0.0] }, "ref_vel": [0.20, 0.0, 0.0], "surface": "LeftFootCenter" },
  			]
  		},

  		"HANDOVER_1stepCycle_back_40cm":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.4, -0.09, 0.0] }, "ref_vel": [-0.20, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.4,   0.09, 0.0] }, "ref_vel": [-0.20, 0.0, 0.0], "surface": "LeftFootCenter"  },
  			]
  		},


  		"HANDOVER_fwd_15cm_steps_30cm_dist":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.15, -0.09, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.3,   0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.3,  -0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" }
  			]
  		},

  		"HANDOVER_back_15cm_steps_30cm_dist":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.15, -0.09, 0.0] }, "ref_vel": [-0.075, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.3,   0.09, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.3, -0.09, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "RightFootCenter" }
  			]
  		},


  		"HANDOVER_fwd_20cm_steps_40cm_dist":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.2, -0.09, 0.0] }, "ref_vel": [0.1, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.4,  0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.4, -0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "RightFootCenter" }
  			]
  		},

  		"HANDOVER_back_20cm_steps_40cm_dist":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.1, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.1, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.2, -0.1, 0.0] }, "ref_vel": [-0.1, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.4,  0.1, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.4, -0.1, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "RightFootCenter" }
  			]
  		},





  		"backward_15cm_steps":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.15, -0.09, 0.0] }, "ref_vel": [-0.075, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.3,   0.09, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.45, -0.09, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.6,   0.09, 0.0] }, "ref_vel": [-0.075, 0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.75, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.75,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  }
  				]
  		},
  		"backward_20cm_steps":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.1, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.1, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.2, -0.1, 0.0] }, "ref_vel": [-0.1, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.4,  0.1, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-0.6, -0.1, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-0.8,  0.1, 0.0] }, "ref_vel": [-0.1, 0.0, 0.0], "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [-1.0, -0.1, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [-1.0,  0.1, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  }
  				]
  		},

  		"comanoid_airbus_staircase": // 1st part of Sep 10, 2019 experiment video (on YouTube)
  		{
  			"init_dsp_duration": 0.6,
  			"single_support_duration": 1.4,
  			"double_support_duration": 0.2,
  			"final_dsp_duration": 0.6,
  			"swing_height": 0.24,
  			"landing_duration": 0.1,
  			"takeoff_duration": 0.42,
  			"torso_pitch": 0.2,
  			"contacts":
  			[
  				{
  					"pose": { "translation": [-0.04,  0.09, 0.000] },
  					"surface": "LeftFootCenter"
  				},
  				{
  					"pose": { "translation": [-0.04, -0.09, 0.000] },
  					"surface": "RightFootCenter",
  					"swing": { "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
  				},
  				{
  					"pose": { "translation": [0.24,   0.09, 0.185] },
  					"surface": "LeftFootCenter",
  					"swing": { "takeoff_offset": [-0.02, 0.0, 0.0] }
  				},
  				{
  					"pose": { "translation": [0.24,  -0.09, 0.185] },
  					"surface": "RightFootCenter",
  					"swing": { "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
  				},
  				{
  					"pose": { "translation": [0.48,   0.09, 0.370] },
  					"surface": "LeftFootCenter",
  					"swing": { "takeoff_offset": [-0.02, 0.0, 0.0] }
  				},
  				{
  					"pose": { "translation": [0.48,  -0.09, 0.370] },
  					"surface": "RightFootCenter",
  					"swing": { "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
  				},
  				{
  					"pose": { "translation": [0.72,   0.09, 0.555] },
  					"surface": "LeftFootCenter",
  					"swing": { "takeoff_offset": [-0.02, 0.0, 0.0] }
  				},
  				{
  					"pose": { "translation": [0.72,  -0.09, 0.555] },
  					"surface": "RightFootCenter",
  					"swing": { "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
  				},
  				{
  					"pose": { "translation": [0.96,   0.09, 0.740] },
  					"surface": "LeftFootCenter",
  					"swing": { "height": 0.2, "takeoff_offset": [-0.02, 0.0, 0.0] }
  				},
  				{
  					"pose": { "translation": [0.96,  -0.09, 0.740] },
  					"surface": "RightFootCenter",
  					"swing": { "height": 0.2, "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
  				},
  				{ "pose": { "translation": [1.20,   0.09, 0.885] }, "surface": "LeftFootCenter"  },
  				{ "pose": { "translation": [1.20,  -0.09, 0.885] }, "surface": "RightFootCenter" }
  			],
  			"mpc":
  			{
  				"weights":
  				{
  					"jerk": 1.0,
  					"vel": [10.0, 300.0],
  					"zmp": 1000.0
  				}
  			}
  		},

  		"comanoid_walk_45cm":  // 2nd part of Sep 10, 2019 experiment video (on YouTube)
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.15, -0.09, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.3,   0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.45, -0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.45,  0.09, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "LeftFootCenter" }
  			]
  		},
  		"forward_15cm_steps":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.15, -0.09, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.3,   0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.45, -0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.6,   0.09, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.75, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.75,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
  				]
  		},

  		"forward_20cm_steps":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.2, -0.09, 0.0] }, "ref_vel": [0.1, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.4,  0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.6, -0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.8,  0.09, 0.0] }, "ref_vel": [0.1, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [1.0, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.0,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" }
  				]
  		},
  		"forward_25cm_steps":
  		{
  			"double_support_duration": 0.1,
  			"single_support_duration": 0.7,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.2, -0.09, 0.0] }, "ref_vel": [0.12, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.45, 0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.7, -0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.95, 0.09, 0.0] }, "ref_vel": [0.12, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [1.15,-0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.15, 0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
  				]
  		},
  		"forward_30cm_steps":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.05,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.2, -0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.5,  0.09, 0.0] }, "ref_vel": [0.30, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.8, -0.09, 0.0] }, "ref_vel": [0.30, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.1,  0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [1.3, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.3,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
  				]
  		},
  		"forward_35cm_steps":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.2,  -0.09, 0.0] }, "ref_vel": [0.17, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.55,  0.09, 0.0] }, "ref_vel": [0.35, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.9,  -0.09, 0.0] }, "ref_vel": [0.35, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.25,  0.09, 0.0] }, "ref_vel": [0.17, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [1.45, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.45,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
  				]
  		},
  		"forward_40cm_steps":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.07,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.2, -0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.6,  0.09, 0.0] }, "ref_vel": [0.4, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [1.0, -0.09, 0.0] }, "ref_vel": [0.4, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.4,  0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [1.6, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.6,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" }
  				]
  		},
  		"kajita_2010":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.2,   -0.09, 0.0] }, "ref_vel": [0.12, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.45,   0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.725, -0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.975,  0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [1.25,  -0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.5,    0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [1.775, -0.09, 0.0] }, "ref_vel": [0.12, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.975,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [1.975, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" }
  				]
  		},
  		"koopa_30cm_steps":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.3, -0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.3,  0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.6, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.6,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
  				]
  		},
  		"koopa_40cm_steps":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.4, -0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.4,  0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.8, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.8,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" }
  				]
  		},
  		"koopa_50cm_steps":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.5, -0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.5,  0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [1.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
  				{ "pose": { "translation": [1.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
  				]
  		},
  		"lateral_left_10cm_steps":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0,  0.09, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.0, -0.09, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.19, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.0,  0.01, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.29, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.0,  0.11, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.39, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.0,  0.21, 0.0] }, "surface": "RightFootCenter" }
  				]
  		},
  		"lateral_right_10cm_steps":
  		{
  			"double_support_duration": 0.2,
  			"single_support_duration": 0.8,
  			"swing_height": 0.04,
  			"contacts":
  			[
  				{ "pose": { "translation": [0.0, -0.09, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0,  0.09, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.0, -0.19, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0, -0.01, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.0, -0.29, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0, -0.11, 0.0] }, "surface": "LeftFootCenter" },
  				{ "pose": { "translation": [0.0, -0.39, 0.0] }, "surface": "RightFootCenter" },
  				{ "pose": { "translation": [0.0, -0.21, 0.0] }, "surface": "LeftFootCenter" }
  				]
  		},
      "warmup":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.035, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.105, 0.0] }, "surface": "LeftFootCenter"  },
          { "pose": { "translation": [0.035, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.105, 0.0] }, "surface": "LeftFootCenter"  }
        ]
      }

    }
	},



	"contacts": [],

	"constraints":
	[
		{
			"type": "contact"
		},
		{
			"type": "kinematics", // constraint type
			"damper": [           // see Equation (3.6) in J. Vaillant's thesis
					0.1,              // interaction distance (d_i)
					0.01,             // safety distance (d_s)
					0.5],             // damper offset (xi_off)
			"robotIndex": 0,      // applies to main robot
		},
		{
			"robotIndex": 2,
			"type": "kinematics",
			// "type": "dynamics",
			// "damper": [0.1, 0.01, 0.5],
			// "infTorque": true
		}
	],

	"collisions":
	[
		{
			"type": "collision",
			"r1Index": 0,
			"r2Index": 0,
			"useMinimal": true
		}
	],

	"robots":
	{
		"ground":
		{
			"module": "env",
			"params": ["@AROBASE@MC_ENV_DESCRIPTION@AROBASE@", "ground"]
		},
		"handoverObjects":
		{
			"module": "object",
			"params": ["@HANDOVER_URDF_PATH@", "handoverPipe"]
			// "params": ["@HANDOVER_URDF_PATH@", "handoverBottle"]
		}
	},

	//
	// mc_control::fsm::Controller configuration
	//

	"init": "Initial",

	"configs": {},

	"states":
  {
		"Walking":
		{
			"base":"Meta",
			"transitions":
			[
				["Standing",      "DoubleSupport", "DoubleSupport"],
				["DoubleSupport", "SingleSupport", "SingleSupport"],
				["DoubleSupport", "Standing",      "Standing"],
				["SingleSupport", "DoubleSupport", "DoubleSupport"]
			]
		},

		"walkingHandover":
		{
			"base":"Parallel",
			"states": ["Walking", "Handover"]
		},
	},

	"triggerWalk" : false,
	"finishedWalk": false,

	"transitions":
	[
		["Initial", "walkingHandover", "walkingHandover"],


		["Initial",       "Standing",      "Standing"],
		["Standing",      "DoubleSupport", "DoubleSupport"],
		["DoubleSupport", "SingleSupport", "SingleSupport"],
		["DoubleSupport", "Standing",      "Standing"],
		["SingleSupport", "DoubleSupport", "DoubleSupport"]
	],

	// Disable floating-base update from MCGlobalController
	"UpdateRealFromSensors": false,

	// When true, the FSM transitions are managed by an external tool
	"Managed": false,

	// When true and the FSM is self-managed, transitions should be triggered
	"StepByStep": false,

	// Where to look for state libraries
	"StatesLibraries": ["@MC_RTC_LIBDIR@/mc_controller/fsm/states", "@STATES_DIR@"],

	// Where to look for state files
	// "StatesFiles": [],
	// "StatesFiles": ["@STATES_DATA_DIR@"],


	// When true, state factory will be more verbose
	"VerboseStateFactory": false,


	// Controller is created in a sandbox, which in between a thread and a fork;
	// try to keep it to false, as it can create weird conflicts with threads
	"UseSandbox" : false,

}
