from schema import Schema, Optional, Regex

map_schema = Schema({
    "nodes": [
        {
            "node_name": str,
            "trigger_radius": float,
            "position":
            {
                "x": float,
                "y": float
            },
            "connections":[str]
        },
    ]
})

travel_plan_schema = Schema({
    "travel_plans": [
        {
            "name": str,
            "loop_plan": bool,
            "checkpoints":[
                {
                    "node_name": str,
                    "actions":
                    {
                        Optional("set_switch_state"): str,
                        Optional("set_speed"): lambda n: 0 <= n <= 100,
                        Optional("set_headlight"): Regex("ON|OFF"),
                        Optional("set_taillight"): Regex("ON|OFF|BREAK"),
                        Optional("set_blinker"): Regex("ON|OFF|HAZARD"),
                    }
                }
            ]
        }
    ]
})