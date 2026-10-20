#include <kilombo.h>
#include <stdbool.h>

#ifdef KILOBOT
#include <avr/io.h>
#endif

#ifdef SIMULATOR
#include <stdio.h>
#endif

#include "abm.h"

REGISTER_USERDATA(USERDATA)

/* ---------- PARAMETERS ---------- */
// Active Brownian Motion parameters

/* ----------- General rules ----------*/

// High Dθ: Increase TUMBLE_PROBABILITY (more erratic)
// Strong repulsion: Decrease REPULSION_DIST, increase AVOIDANCE_TURN_MAX
// Ballistic motion: Decrease TUMBLE_PROBABILITY, increase FORWARD_TIME_MEAN

#define FORWARD_TIME_MEAN   40    // Mean forward time (~5 sec) (Self-propulsion effect)
#define FORWARD_TIME_VAR    40     // Variation in forward time

// Rotational diffusion (random tumbling)
#define TUMBLE_CHECK_INTERVAL  10  // Check every ~0.5 sec if we should tumble
#define TUMBLE_PROBABILITY     60  // % chance to tumble per check (simulates Dθ) (more erratic)
#define TUMBLE_DURATION_MIN    50  // Short random turns
#define TUMBLE_DURATION_MAX    96

// Repulsion parameters
#define REPULSION_DIST_NEAR    50  // Close range - turn away immediately
#define AVOIDANCE_TURN_MIN     20  // Longer turn for collision avoidance
#define AVOIDANCE_TURN_MAX     50

/* ---------- HELPER: smooth motors ---------- */
void smooth_set_motors(uint8_t ccw, uint8_t cw)
{
#ifdef KILOBOT
    uint8_t l = 0, r = 0;
    if (ccw && !OCR2A) l = 0xff;
    if (cw && !OCR2B) r = 0xff;
    if (l || r) {
        set_motors(l, r);
        delay(15);
    }
#endif
    set_motors(ccw, cw);
}

/* ---------- MOTION HELPERS ---------- */
void start_forward()
{
    smooth_set_motors(kilo_straight_left, kilo_straight_right);
    set_color(RGB(0,0,0));   // Blue
    mydata->state = FORWARD;
    mydata->state_start_time = kilo_ticks;
    
    // Set random forward duration (variability in persistence)
    mydata->forward_duration = FORWARD_TIME_MEAN + 
        (rand_soft() % FORWARD_TIME_VAR) - (FORWARD_TIME_VAR/2);
    
    mydata->last_tumble_check = kilo_ticks;
}

void start_tumble()
{
    // Random tumble (rotational diffusion Dθ)
    set_color(RGB(0,0,0));   // Green
    
    if (rand_soft() & 1)
        smooth_set_motors(kilo_turn_left, 0);
    else
        smooth_set_motors(0, kilo_turn_right);
    
    mydata->turn_duration = 
        TUMBLE_DURATION_MIN + (rand_soft() % (TUMBLE_DURATION_MAX - TUMBLE_DURATION_MIN));
    mydata->state_start_time = kilo_ticks;
    mydata->state = TURN;
    mydata->turn_type = TUMBLE;
}

void start_avoidance_turn()
{
    // Repulsion-induced turn (force from neighbors)
    set_color(RGB(0,0,0));   // RED
    
    if (rand_soft() & 1)
        smooth_set_motors(kilo_turn_left, 0);
    else
        smooth_set_motors(0, kilo_turn_right);
    
    mydata->turn_duration = 
        AVOIDANCE_TURN_MIN + (rand_soft() % (AVOIDANCE_TURN_MAX - AVOIDANCE_TURN_MIN));
    mydata->state_start_time = kilo_ticks;
    mydata->state = TURN;
    mydata->turn_type = AVOIDANCE;
}

/* ---------- MAIN LOOP ---------- */
void loop()
{
    // Update distance estimate if a new message arrived
    if (mydata->new_message) {
        mydata->new_message = 0;
        uint8_t dist = estimate_distance(&mydata->dist);
        
        // Store closest neighbor distance for repulsion
        if (dist && (mydata->nearest_neighbor == 0 || dist < mydata->nearest_neighbor)) {
            mydata->nearest_neighbor = dist;
        }
    }
    
    switch (mydata->state) {
    case FORWARD:
        // REPULSION: Immediate avoidance if too close
        if (mydata->nearest_neighbor && mydata->nearest_neighbor < REPULSION_DIST_NEAR) {
            mydata->nearest_neighbor = 0;  // Reset
            start_avoidance_turn();
            return;
        }
        
        // ROTATIONAL DIFFUSION: Random tumbling
        if (kilo_ticks - mydata->last_tumble_check > TUMBLE_CHECK_INTERVAL) {
            mydata->last_tumble_check = kilo_ticks;
            
            mydata->nearest_neighbor = 0;  // Reset for next interval
            
            // Probabilistic tumble (pure Dθ, independent of neighbors)
            if ((rand_soft() % 100) < TUMBLE_PROBABILITY) {
                start_tumble();
                return;
            }
        }
        
        // Natural end of forward motion (reset trajectory)
        if (kilo_ticks - mydata->state_start_time > mydata->forward_duration) {
            start_tumble();
        }
        break;
        
    case TURN:
        // Turn complete - return to self-propulsion
        if (kilo_ticks - mydata->state_start_time > mydata->turn_duration) {
            start_forward();
        }
        break;
    }
}

/* ---------- MESSAGING ---------- */
void message_rx(message_t *m, distance_measurement_t *d)
{
    mydata->new_message = 1;
    mydata->dist = *d;
}

message_t *message_tx()
{
    return &mydata->transmit_msg;
}

/* ---------- SETUP ---------- */
void setup()
{
    mydata->state = FORWARD;
    mydata->nearest_neighbor = 0;
    mydata->new_message = 0;
    mydata->turn_type = TUMBLE;
    mydata->last_tumble_check = 0;
    
    mydata->transmit_msg.type = NORMAL;
    mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
    
    rand_seed(kilo_uid);
    start_forward();
}

#ifdef SIMULATOR
/* status bar info */
static char botinfo_buffer[256];
char *cb_botinfo(void)
{
    char *p = botinfo_buffer;
    p += sprintf(p, "ID: %d\n", kilo_uid);
    
    if (mydata->state == FORWARD)
        p += sprintf(p, "State: FORWARD (v₀e(θ))\n");
    else if (mydata->turn_type == TUMBLE)
        p += sprintf(p, "State: TUMBLE (Dθ)\n");
    else
        p += sprintf(p, "State: AVOID (repulsion)\n");
    
    if (mydata->nearest_neighbor)
        p += sprintf(p, "Nearest: %d mm\n", mydata->nearest_neighbor);
    
    return botinfo_buffer;
}
#endif

int main()
{
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    
#ifdef SIMULATOR
    SET_CALLBACK(botinfo, cb_botinfo);
#endif
    
    kilo_start(setup, loop);
    return 0;
}