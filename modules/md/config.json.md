{
    "ip_address": "192.168.250.1",
    "port": 1502,
    "period_s": 0.1,
    "interpolar_points": 4,
    "object_types": {
        "object_A": "object_A"
    },
    "object_A": [140.0, -120.0, -205.0],
    "scheduler": {
        "home_position": [0.0, 0.0, -180.0],
        "clearance_height": -165.0,
        "pickup_height": -230.0,
        "pre_pick_height": -210.0,
        "place_height": -205.0,
        "corner_blend_xy": 35.0,
        "intercept_lead_time_s": 0.14,
        "release_descent_time_s": 0.14,
        "nominal_xy_speed": 220.0,
        "nominal_z_speed": 180.0,
        "stale_timeout_s": 5.0,
        "speed_timeout_s": 1.0,
        "poll_interval_s": 0.05,
        "default_speed": 80.0,
        "robot_movement_delay_s": 0.05,
        "ethernet_delay_s": 0.002,
        "pickup_window_x": [-120.0, 120.0],
        "pickup_window_y": [-120.0, 120.0],
        "throughput_object_types": ["object_A"],
        "throughput_lanes": [-60.0, 0.0, 60.0],
        "throughput_spawn_x": -180.0,
        "throughput_emit_interval_s": 0.35,
        "accuracy_emit_interval_s": 0.8,
        "execution_margin_s": 0.3,
        "accuracy_points": [
            [40.0, -60.0, -220.0],
            [0.0, 0.0, -220.0],
            [-40.0, 60.0, -220.0]
        ],
        "log_path": "data.log"
    }
}
