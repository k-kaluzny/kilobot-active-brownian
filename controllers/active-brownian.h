#ifndef ABM_H
#define ABM_H

// State definitions
typedef enum {
    FORWARD,
    TURN
} state_t;

// Turn type
typedef enum {
    TUMBLE,      // Random rotational diffusion
    AVOIDANCE    // Repulsion from neighbors
} turn_type_t;

// User data structure
typedef struct {
    state_t state;
    turn_type_t turn_type;
    uint32_t state_start_time;
    uint32_t last_tumble_check;
    uint16_t turn_duration;
    uint16_t forward_duration;
    uint8_t nearest_neighbor;
    uint8_t new_message;
    distance_measurement_t dist;
    message_t transmit_msg;
} USERDATA;

#endif