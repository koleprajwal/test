#include "step_tracker.h"
#include <math.h>
#include <string.h>

#define DT_SEC                 0.010f
#define G_MPS2                 9.80665f
#define PI_F                   3.14159265358979323846f
#define STEP_PEAK_THRESHOLD_G  0.22f
#define MIN_EVENT_INTERVAL_MS  300U
#define STANCE_GYRO_DPS        35.0f
#define STANCE_ACCEL_ERROR_G   0.18f
#define STANCE_SAMPLES_REQUIRED 5U
#define ASSUMED_STRIDE_M       0.65f

static float clampf_local(float x, float lo, float hi) { return (x < lo) ? lo : ((x > hi) ? hi : x); }

void StepTracker_Init(StepTracker *s)
{
    memset(s, 0, sizeof(*s));
}

void StepTracker_Update(StepTracker *s, const ICM20608_Data *d, uint32_t now_ms)
{
    static uint8_t initialized = 0U;
    static float roll = 0.0f, pitch = 0.0f;
    static float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    static float prev_motion = 0.0f, prev_prev_motion = 0.0f;
    static float peak_candidate = 0.0f;
    static uint32_t last_event_ms = 0U;
    static uint8_t stance_count = 0U;
    static uint8_t had_swing = 0U;
    static float gravity_norm_lp = 1.0f;

    s->valid_event = 0U;
    if (!initialized) {
        roll = atan2f(d->ay_g, d->az_g);
        pitch = atan2f(-d->ax_g, sqrtf(d->ay_g*d->ay_g + d->az_g*d->az_g));
        initialized = 1U;
    }

    /* Complementary tilt estimate; yaw is gyro-integrated and will drift. */
    {
        float roll_acc = atan2f(d->ay_g, d->az_g);
        float pitch_acc = atan2f(-d->ax_g, sqrtf(d->ay_g*d->ay_g + d->az_g*d->az_g));
        roll = 0.985f * (roll + d->gx_dps * (PI_F/180.0f) * DT_SEC) + 0.015f * roll_acc;
        pitch = 0.985f * (pitch + d->gy_dps * (PI_F/180.0f) * DT_SEC) + 0.015f * pitch_acc;
        s->yaw_rad += d->gz_dps * (PI_F/180.0f) * DT_SEC;
        if (s->yaw_rad > PI_F) s->yaw_rad -= 2.0f*PI_F;
        if (s->yaw_rad < -PI_F) s->yaw_rad += 2.0f*PI_F;
    }

    /* Rotate body acceleration into a world frame using roll/pitch/yaw. */
    {
        float cr=cosf(roll), sr=sinf(roll), cp=cosf(pitch), sp=sinf(pitch);
        float cy=cosf(s->yaw_rad), sy=sinf(s->yaw_rad);
        float ax=d->ax_g, ay=d->ay_g, az=d->az_g;
        float r00=cy*cp, r01=cy*sp*sr-sy*cr, r02=cy*sp*cr+sy*sr;
        float r10=sy*cp, r11=sy*sp*sr+cy*cr, r12=sy*sp*cr-cy*sr;
        float r20=-sp,   r21=cp*sr,              r22=cp*cr;
        float awx=(r00*ax+r01*ay+r02*az)*G_MPS2;
        float awy=(r10*ax+r11*ay+r12*az)*G_MPS2;
        float awz=(r20*ax+r21*ay+r22*az)*G_MPS2 - G_MPS2;
        float amag=sqrtf(ax*ax+ay*ay+az*az);
        float gmag=sqrtf(d->gx_dps*d->gx_dps+d->gy_dps*d->gy_dps+d->gz_dps*d->gz_dps);
        float motion;
        uint8_t stance_now;

        /* Crude stance detector for zero-velocity correction; tune after logs. */
        gravity_norm_lp = 0.90f*gravity_norm_lp + 0.10f*amag;
        stance_now = (gmag < STANCE_GYRO_DPS && fabsf(gravity_norm_lp-1.0f) < STANCE_ACCEL_ERROR_G) ? 1U : 0U;
        if (stance_now) { if (stance_count < 255U) stance_count++; } else { stance_count=0U; had_swing=1U; }

        if (stance_count >= STANCE_SAMPLES_REQUIRED) {
            /* ZUPT: foot believed stationary; suppress accumulated velocity. */
            vx=0.0f; vy=0.0f; vz=0.0f;
        } else {
            vx += awx*DT_SEC; vy += awy*DT_SEC; vz += awz*DT_SEC;
            s->x_m += vx*DT_SEC; s->y_m += vy*DT_SEC; s->z_m += vz*DT_SEC;
        }

        /* First-pass peak detector. It is a tunable prototype, not validated gait classification. */
        motion = fabsf(amag - gravity_norm_lp);
        motion = clampf_local(motion, 0.0f, 3.0f);
        if (motion > peak_candidate) peak_candidate = motion;

        /* A local maximum is identified when the signal has begun falling. */
        if ((prev_motion > prev_prev_motion) && (prev_motion >= motion) &&
            (prev_motion >= STEP_PEAK_THRESHOLD_G) &&
            ((now_ms - last_event_ms) >= MIN_EVENT_INTERVAL_MS)) {
            s->observed_leg_events++;
            /* A same-leg gait-cycle event roughly spans two alternating footfalls. */
            s->estimated_total_steps = s->observed_leg_events * 2U;
            s->event_time_ms = now_ms;
            s->valid_event = 1U;
            last_event_ms = now_ms;
            peak_candidate = 0.0f;
            had_swing = 0U;
        }
        prev_prev_motion = prev_motion;
        prev_motion = motion;
        (void)had_swing;
    }
}
