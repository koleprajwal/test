#ifndef STEP_TRACKER_H
#define STEP_TRACKER_H
#include "icm20608.h"
#include <stdint.h>

typedef struct {
    uint32_t observed_leg_events;
    uint32_t estimated_total_steps;
    uint32_t event_time_ms;
    float x_m, y_m, z_m;
    float yaw_rad;
    uint8_t valid_event;
} StepTracker;

void StepTracker_Init(StepTracker *s);
/* Call exactly once per nominal 10 ms sample. Position is an IMU-only estimate. */
void StepTracker_Update(StepTracker *s, const ICM20608_Data *d, uint32_t now_ms);

#endif
