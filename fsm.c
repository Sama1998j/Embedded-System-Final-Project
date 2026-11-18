#include <xc.h>
#include <math.h>
#include "fsm.h"
#include "motor.h"
#include "lights.h"
#include "adc.h"

// Protect shared FSM state from the RE8 interrupt
static inline void state_lock(void)
{
    IEC1bits.INT1IE = 0;
}

static inline void state_unlock(void)
{
    IEC1bits.INT1IE = 1;
}

#define STATE_HALTED    0
#define STATE_MOVING    1
#define STATE_OBSTACLE  2
#define STATE_EMERGENCY 3

// Obstacle-avoidance sub-phases: rotate, advance, rotate back.
#define OBS_ROTATE_CW  0
#define OBS_FORWARD    1
#define OBS_ROTATE_ACW 2

#define THRESHOLD_CM_DEFAULT   20      // default obstacle trigger distance
#define MANEUVER_MIN   40      // since a speed lower than 40% will not rotate buggy
#define TURN_TARGET    90.0    // degrees to rotate
#define FORWARD_TICKS  1000    // 2s at 500 Hz
#define MAX_ATTEMPTS   3

// 1 Hz blink at 500 Hz loop
#define BLINK_PERIOD   500
#define BLINK_HALF     250


// Shared between main loop and RE8 ISR
static volatile int state = STATE_HALTED;
static volatile int previous_state = STATE_HALTED;  // state to resume after $PCEMR,off*
static int blink_counter = 0;


// Runtime-configurable obstacle threshold, set via $PCCFG,THRESHOLD,x*
static int threshold_cm = THRESHOLD_CM_DEFAULT;

// Obstacle-avoidance vars
static int obs_phase;
static int obs_attempts;
static int forward_counter;
static int maneuver_speed;
static double phase_start_yaw;

// Re-anchor obstacle avoidance after Emergency
static volatile int resume_pending = 0;


// Angular difference normalised to [-180, 180].
static double angle_diff(double a, double b)
{
    double d = a - b;
    while (d > 180.0)  d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

void fsm_init(void)
{
    state = STATE_HALTED;
    previous_state = STATE_HALTED;
    blink_counter = 0;
    resume_pending = 0;
}

// Immediate RE8 emergency stop during Obstacle state
void fsm_emergency_from_isr(void)
{
    if (state == STATE_OBSTACLE)
    {
        previous_state = STATE_OBSTACLE;
        state = STATE_EMERGENCY;
        motor_set(0, 0);   // stop immediately, don't wait for the next fsm_update
    }
}

// Halted/Moving toggle after RE8 debounce
void fsm_request_toggle(void)
{
    state_lock();
    if (state == STATE_HALTED)
    {
        state = STATE_MOVING;
    }
    else if (state == STATE_MOVING)
    {
        state = STATE_HALTED;
    }
    state_unlock();
}

void fsm_emergency_clear(void)
{
    state_lock();
    if (state == STATE_EMERGENCY)
    {
        resume_pending = 1;
        state = previous_state;
    }
    state_unlock();
}
// Limit threshold to the valid IR range
void fsm_set_threshold(int cm)
{
    if (cm < ADC_DIST_MIN_CM) cm = ADC_DIST_MIN_CM;
    if (cm > ADC_DIST_MAX_CM) cm = ADC_DIST_MAX_CM;
    threshold_cm = cm;
}

static void start_obstacle(double yaw, int speed)
{
    // Initialize obstacle-avoidance variables
    obs_phase = OBS_ROTATE_CW;
    obs_attempts = 1;
    phase_start_yaw = yaw;

    // Manoeuvre at the current speed, floored so the wheels turn.
    maneuver_speed = speed;
    if (maneuver_speed < 0)
    {
        maneuver_speed = -maneuver_speed;
    }
    if (maneuver_speed < MANEUVER_MIN)
    {
        maneuver_speed = MANEUVER_MIN;
    }
}

void fsm_update(pc_reference ref[], int distance, double yaw)
{
    int blink_on;

    // Read shared FSM state atomically
    state_lock();
    int state_at_entry = state;
    int local_state = state_at_entry;
    state_unlock();

    blink_counter++;
    if (blink_counter >= BLINK_PERIOD)
    {
        blink_counter = 0;
        }
        blink_on = (blink_counter < BLINK_HALF);

        if (local_state == STATE_HALTED)
    {
        motor_set(0, 0);
        lights_left(blink_on);
        lights_right(blink_on);
        lights_low(0);
    }
    else if (local_state == STATE_MOVING)
    {
        motor_set(ref[0].speed - ref[0].yawrate,
                  ref[0].speed + ref[0].yawrate);
        lights_left(0);
        lights_right(0);
        lights_low(1);

        if (distance < threshold_cm)
        {
            start_obstacle(yaw, ref[0].speed);
            local_state = STATE_OBSTACLE;
        }
    }
    else if (local_state == STATE_OBSTACLE)
    {
        // Re-anchor the turn reference after an emergency resume
        if (resume_pending)
        {
            resume_pending = 0;
            phase_start_yaw = yaw;
        }

        // right light blinks 1 Hz, low on, left off
        lights_left(0);
        lights_right(blink_on);
        lights_low(1);

        if (obs_phase == OBS_ROTATE_CW)
        {
            motor_set(maneuver_speed, -maneuver_speed);   // clockwise
            if (fabs(angle_diff(yaw, phase_start_yaw)) >= TURN_TARGET)
            {
                obs_phase = OBS_FORWARD;
                forward_counter = 0;
            }
        }
        else if (obs_phase == OBS_FORWARD)
        {
            motor_set(maneuver_speed, maneuver_speed);    // forward
            forward_counter++;

            if (distance < threshold_cm)
            {
                local_state = STATE_HALTED;
            }
            else if (forward_counter >= FORWARD_TICKS)
            {
                obs_phase = OBS_ROTATE_ACW;
                phase_start_yaw = yaw;
            }
        }
        else // OBS_ROTATE_ACW
        {
            motor_set(-maneuver_speed, maneuver_speed);   // anticlockwise, back to heading
            if (fabs(angle_diff(yaw, phase_start_yaw)) >= TURN_TARGET)
            {
                if (distance >= threshold_cm)
                {
                    local_state = STATE_MOVING;
                }
                else if (obs_attempts >= MAX_ATTEMPTS)
                {
                    local_state = STATE_HALTED;
                }
                else
                {
                    obs_attempts++;
                    obs_phase = OBS_ROTATE_CW;
                    phase_start_yaw = yaw;
                }
            }
        }
    }
    else // STATE_EMERGENCY
    {
        motor_set(0, 0);
        lights_left(0);
        lights_right(0);
        lights_low(0);
    }

    // Commit state only if it was not changed by the ISR
    state_lock();
    if (state == state_at_entry)
    {
        state = local_state;
    }
    state_unlock();
}