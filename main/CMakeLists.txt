idf_component_register(
    SRCS 
        "main.c"
        "dog/dog_config.c"
        "dog/dog_imu.c"
        # Reaction system (user interaction animations)
        "reaction/reaction_config.c"
        "reaction/walk_forward_reaction.c"
        "reaction/gyro_balance.c"
        # Different gait implementations
        "gaits/trot_gait.c"
        "gaits/walk_gait.c"
        "gaits/creep_gait.c"
        "gaits/crawl_gait.c"
        # Minimal BLE servo control
        "ble/ble_servo.c"
        # Optional local utilities
        "util/sts3032_config.c"
        
        # Optional: Uncomment to include additional examples
        # "examples/example_modes.c"
        
    INCLUDE_DIRS 
        "."
        "dog"
        "reaction"
        "gaits"
        "util"
        "examples"
        "ble"
    
    REQUIRES
        sts3032
        bt
        json
        nvs_flash
        qmi8658a
)
