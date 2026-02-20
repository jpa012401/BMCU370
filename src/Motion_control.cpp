#include "Motion_control.h"

AS5600_soft_IIC_many MC_AS5600;
uint32_t AS5600_SCL[] = {PB15, PB14, PB13, PB12};
uint32_t AS5600_SDA[] = {PD0, PC15, PC14, PC13};
#define AS5600_PI 3.1415926535897932384626433832795
#define speed_filter_k 100
float speed_as5600[4] = {0, 0, 0, 0};

void MC_PULL_ONLINE_init()
{
    ADC_DMA_init();
}
float MC_PULL_stu_raw[4] = {0, 0, 0, 0};
int MC_PULL_stu[4] = {0, 0, 0, 0};
float MC_ONLINE_key_stu_raw[4] = {0, 0, 0, 0};
// 0-offline 1-online dual micro-switch triggered 2-outer triggered 3-inner triggered
int MC_ONLINE_key_stu[4] = {0, 0, 0, 0};

// Voltage control related constants
float PULL_voltage_up = 1.85f;   // State: high pressure, red light
float PULL_voltage_down = 1.45f; // State: low pressure, blue light
#define PULL_VOLTAGE_SEND_MAX 1.7f
// Micro-switch trigger control related constants
bool Assist_send_filament[4] = {false, false, false, false};
bool pull_state_old = false; // Previous trigger state - True: not triggered, False: feeding complete
bool is_backing_out = false;
uint64_t Assist_filament_time[4] = {0, 0, 0, 0};
uint64_t Assist_send_time = 1200; // Feed duration after only outer trigger
// Unload distance in MM
float_t P1X_OUT_filament_meters = 200.0f; // Internal 200mm, external 700mm
float_t last_total_distance[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // Initialize distance at unload start
// bool filament_channel_inserted[4]={false,false,false,false}; // Whether channel is inserted
// Use dual micro-switch
#define is_two false

void MC_PULL_ONLINE_read()
{
    float *data = ADC_DMA_get_value();
    MC_PULL_stu_raw[3] = data[0];
    MC_ONLINE_key_stu_raw[3] = data[1];
    MC_PULL_stu_raw[2] = data[2];
    MC_ONLINE_key_stu_raw[2] = data[3];
    MC_PULL_stu_raw[1] = data[4];
    MC_ONLINE_key_stu_raw[1] = data[5];
    MC_PULL_stu_raw[0] = data[6];
    MC_ONLINE_key_stu_raw[0] = data[7];

    for (int i = 0; i < 4; i++)
    {
        /*
        if (i == 0){
            DEBUG_MY("MC_PULL_stu_raw = ");
            DEBUG_float(MC_PULL_stu_raw[i],3);
            DEBUG_MY("  MC_ONLINE_key_stu_raw = ");
            DEBUG_float(MC_ONLINE_key_stu_raw[i],3);
            DEBUG_MY("  Channel:");
            DEBUG_float(i,1);
            DEBUG_MY("   \n");
        }
        */
        if (MC_PULL_stu_raw[i] > PULL_voltage_up) // Greater than 1.85V, pressure too high
        {
            MC_PULL_stu[i] = 1;
        }
        else if (MC_PULL_stu_raw[i] < PULL_voltage_down) // Less than 1.45V, pressure too low
        {
            MC_PULL_stu[i] = -1;
        }
        else // Between 1.4~1.7, within normal tolerance, no action needed
        {
            MC_PULL_stu[i] = 0;
        }
        /* Online status */

        // Filament online detection
        if (is_two == false)
        {
            // Greater than 1.65V means filament online, high level
            if (MC_ONLINE_key_stu_raw[i] > 1.65)
            {
                MC_ONLINE_key_stu[i] = 1;
            }
            else
            {
                MC_ONLINE_key_stu[i] = 0;
            }
        }
        else
        {
            // DEBUG_MY(MC_ONLINE_key_stu_raw);
            // Dual micro-switch
            if (MC_ONLINE_key_stu_raw[i] < 0.6f)
            { // Less than means offline
                MC_ONLINE_key_stu[i] = 0;
            }
            else if ((MC_ONLINE_key_stu_raw[i] < 1.7f) & (MC_ONLINE_key_stu_raw[i] > 1.4f))
            { // Only outer micro-switch triggered, needs assisted feeding
                MC_ONLINE_key_stu[i] = 2;
            }
            else if (MC_ONLINE_key_stu_raw[i] > 1.7f)
            { // Both micro-switches triggered, online status
                MC_ONLINE_key_stu[i] = 1;
            }
            else if (MC_ONLINE_key_stu_raw[i] < 1.4f)
            { // Only inner micro-switch triggered, need to confirm if out of material or vibration
                MC_ONLINE_key_stu[i] = 3;
            }
        }
    }
}

#define PWM_lim 1000

struct alignas(4) Motion_control_save_struct
{
    int Motion_control_dir[4];
    int check = 0x40614061;
} Motion_control_data_save;

#define Motion_control_save_flash_addr ((uint32_t)0x0800E000)
bool Motion_control_read()
{
    Motion_control_save_struct *ptr = (Motion_control_save_struct *)(Motion_control_save_flash_addr);
    if (ptr->check == 0x40614061)
    {
        memcpy(&Motion_control_data_save, ptr, sizeof(Motion_control_save_struct));
        return true;
    }
    return false;
}
void Motion_control_save()
{
    Flash_saves(&Motion_control_data_save, sizeof(Motion_control_save_struct), Motion_control_save_flash_addr);
}

class MOTOR_PID
{

    float P = 0;
    float I = 0;
    float D = 0;
    float I_save = 0;
    float E_last = 0;
    float pid_MAX = PWM_lim;
    float pid_MIN = -PWM_lim;
    float pid_range = (pid_MAX - pid_MIN) / 2;

public:
    MOTOR_PID()
    {
        pid_MAX = PWM_lim;
        pid_MIN = -PWM_lim;
        pid_range = (pid_MAX - pid_MIN) / 2;
    }
    MOTOR_PID(float P_set, float I_set, float D_set)
    {
        init_PID(P_set, I_set, D_set);
        pid_MAX = PWM_lim;
        pid_MIN = -PWM_lim;
        pid_range = (pid_MAX - pid_MIN) / 2;
    }
    void init_PID(float P_set, float I_set, float D_set) // Note: uses independent PID calculation, I and D are already multiplied by P
    {
        P = P_set;
        I = I_set;
        D = D_set;
        I_save = 0;
        E_last = 0;
    }
    float caculate(float E, float time_E)
    {
        I_save += I * E * time_E;
        if (I_save > pid_range) // Limit I
            I_save = pid_range;
        if (I_save < -pid_range)
            I_save = -pid_range;

        float ouput_buf;
        if (time_E != 0) // Prevent rapid calls
            ouput_buf = P * E + I_save + D * (E - E_last) / time_E;
        else
            ouput_buf = P * E + I_save;

        if (ouput_buf > pid_MAX)
            ouput_buf = pid_MAX;
        if (ouput_buf < pid_MIN)
            ouput_buf = pid_MIN;

        E_last = E;
        return ouput_buf;
    }
    void clear()
    {
        I_save = 0;
        E_last = 0;
    }
};

enum class filament_motion_enum
{
    filament_motion_send,
    filament_motion_redetect,
    filament_motion_slow_send,
    filament_motion_pull,
    filament_motion_stop,
    filament_motion_pressure_ctrl_on_use,
    filament_motion_pressure_ctrl_idle,
};
enum class pressure_control_enum
{
    less_pressure,
    all,
    over_pressure
};

class _MOTOR_CONTROL
{
public:
    filament_motion_enum motion = filament_motion_enum::filament_motion_stop;
    int CHx = 0;
    uint64_t motor_stop_time = 0;
    MOTOR_PID PID_speed = MOTOR_PID(2, 20, 0);
    MOTOR_PID PID_pressure = MOTOR_PID(1500, 0, 0);
    float pwm_zero = 500;
    float dir = 0;
    int x1 = 0;
    _MOTOR_CONTROL(int _CHx)
    {
        CHx = _CHx;
        motor_stop_time = 0;
        motion = filament_motion_enum::filament_motion_stop;
    }

    void set_pwm_zero(float _pwm_zero)
    {
        pwm_zero = _pwm_zero;
    }
    void set_motion(filament_motion_enum _motion, uint64_t over_time)
    {
        uint64_t time_now = get_time64();
        motor_stop_time = time_now + over_time;
        if (motion != _motion)
        {
            motion = _motion;
            PID_speed.clear();
        }
    }
    filament_motion_enum get_motion()
    {
        return motion;
    }
    float _get_x_by_pressure(float pressure_voltage, float control_voltage, float time_E, pressure_control_enum control_type)
    {
        float x=0;
        switch (control_type)
        {
        case pressure_control_enum::all: // Full range control
        {
            x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            break;
        }
        case pressure_control_enum::less_pressure: // Low pressure control only
        {
            if (pressure_voltage < control_voltage)
            {
                x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            }
            break;
        }
        case pressure_control_enum::over_pressure: // High pressure control only
        {
            if (pressure_voltage > control_voltage)
            {
                x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            }
            break;
        }
        }
        if (x > 0) // Convert control force to squared enhancement, squaring removes sign so need to check
            x = x * x / 250;
        else
            x = -x * x / 250;
        return x;
    }
    void run(float time_E)
    {
        // When in unload state and unloading is needed, start recording distance
        if (is_backing_out){
            last_total_distance[CHx] += fabs(speed_as5600[CHx] * time_E);
        }
        float speed_set = 0;
        float now_speed = speed_as5600[CHx];
        float x=0;

        uint16_t device_type = get_now_BambuBus_device_type();
        static uint64_t countdownStart[4] = {0};          // Assisted feeding countdown
        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle) // In idle state
        {
            // When both micro-switches are released
            if (MC_ONLINE_key_stu[CHx] == 0)
            {
                Assist_send_filament[CHx] = true; // Can trigger assisted feeding once after channel goes offline
                countdownStart[CHx] = 0;          // Clear countdown
            }

            if (Assist_send_filament[CHx] && is_two)
            { // Allowed state, attempt assisted feeding
                if (MC_ONLINE_key_stu[CHx] == 2)
                {                   // Outer micro-switch triggered
                    x = -dir * 666; // Drive feeding
                }
                if (MC_ONLINE_key_stu[CHx] == 1)
                { // Both micro-switches triggered, prepare to stop
                    if (countdownStart[CHx] == 0)
                    { // Start countdown
                        countdownStart[CHx] = get_time64();
                    }
                    uint64_t now = get_time64();
                    if (now - countdownStart[CHx] >= Assist_send_time) // Countdown
                    {
                        x = 0;                             // Stop motor
                        Assist_send_filament[CHx] = false; // Condition met, assisted feeding round complete
                    }
                    else
                    {
                        // Drive feeding
                        x = -dir * 666;
                    }
                }
            }
            else
            {
                // Already triggered, or micro-switch in other state
                if (MC_ONLINE_key_stu[CHx] != 0 && MC_PULL_stu[CHx] != 0)
                { // If slider is manually pulled, respond accordingly
                    x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - 1.65, time_E);
                }
                else
                { // Otherwise, keep stopped
                    x = 0;
                    PID_pressure.clear();
                }
            }
        }
        else if (MC_ONLINE_key_stu[CHx] != 0) // Channel running and has filament
        {
            if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) // In use state
            {
                if (pull_state_old) { // First time entering use, don't trigger retract, flushing will reset buffer
                    if (MC_PULL_stu_raw[CHx] < 1.55){
                        pull_state_old = false; // Detected filament at low pressure
                    }
                } else {
                    if (MC_PULL_stu_raw[CHx] < 1.65)
                    {
                        x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.65, time_E, pressure_control_enum::less_pressure);
                    }
                    else if (MC_PULL_stu_raw[CHx] > 1.7)
                    {
                        x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.7, time_E, pressure_control_enum::over_pressure);
                    }
                }
            }
            else
            {
                if (motion == filament_motion_enum::filament_motion_stop) // Request stop
                {
                    PID_speed.clear();
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }
                if (motion == filament_motion_enum::filament_motion_send) // Feeding
                {
                    if (device_type == BambuBus_AMS_lite)
                    {
                        if (MC_PULL_stu_raw[CHx] < PULL_VOLTAGE_SEND_MAX) // Pressure actively reaches this position
                            speed_set = 30;
                        else
                            speed_set = 0; // Original was 10 here
                    }
                    else
                    {
                        speed_set = 50; // P series full speed
                    }
                }
                if (motion == filament_motion_enum::filament_motion_slow_send) // Request slow feeding
                {
                    speed_set = 3;
                }
                if (motion == filament_motion_enum::filament_motion_pull) // Retract
                {
                    speed_set = -50;
                }
                x = dir * PID_speed.caculate(now_speed - speed_set, time_E);
            }
        }
        else // Filament ran out during operation, need to stop motor control
        {
            x = 0;
        }

        if (x > 10)
            x += pwm_zero;
        else if (x < -10)
            x -= pwm_zero;
        else
            x = 0;

        if (x > PWM_lim)
        {
            x = PWM_lim;
        }
        if (x < -PWM_lim)
        {
            x = -PWM_lim;
        }

        Motion_control_set_PWM(CHx, x);
    }
};
_MOTOR_CONTROL MOTOR_CONTROL[4] = {_MOTOR_CONTROL(0), _MOTOR_CONTROL(1), _MOTOR_CONTROL(2), _MOTOR_CONTROL(3)};

void Motion_control_set_PWM(uint8_t CHx, int PWM) // Pass PWM to hardware layer for motor control
{
    uint16_t set1 = 0, set2 = 0;
    if (PWM > 0)
    {
        set1 = PWM;
    }
    else if (PWM < 0)
    {
        set2 = -PWM;
    }
    else // PWM==0
    {
        set1 = 1000;
        set2 = 1000;
    }
    switch (CHx)
    {
    case 3:
        TIM_SetCompare1(TIM2, set1);
        TIM_SetCompare2(TIM2, set2);
        break;
    case 2:
        TIM_SetCompare1(TIM3, set1);
        TIM_SetCompare2(TIM3, set2);
        break;
    case 1:
        TIM_SetCompare1(TIM4, set1);
        TIM_SetCompare2(TIM4, set2);
        break;
    case 0:
        TIM_SetCompare3(TIM4, set1);
        TIM_SetCompare4(TIM4, set2);
        break;
    }
}

int32_t as5600_distance_save[4] = {0, 0, 0, 0};
void AS5600_distance_updata() // Read AS5600 and update related data
{
    static uint64_t time_last = 0;
    uint64_t time_now;
    float T;
    do
    {
        time_now = get_time64();
    } while (time_now <= time_last); // T!=0
    T = (float)(time_now - time_last);
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        if ((MC_AS5600.online[i] == false))
        {
            as5600_distance_save[i] = 0;
            speed_as5600[i] = 0;
            continue;
        }

        int32_t cir_E = 0;
        int32_t last_distance = as5600_distance_save[i];
        int32_t now_distance = MC_AS5600.raw_angle[i];
        float distance_E;
        if ((now_distance > 3072) && (last_distance <= 1024))
        {
            cir_E = -4096;
        }
        else if ((now_distance <= 1024) && (last_distance > 3072))
        {
            cir_E = 4096;
        }

        distance_E = -(float)(now_distance - last_distance + cir_E) * AS5600_PI * 7.5 / 4096; // D=7.5mm, negative sign because AS5600 faces the magnet
        as5600_distance_save[i] = now_distance;

        float speedx = distance_E / T * 1000;
        // T = speed_filter_k / (T + speed_filter_k);
        speed_as5600[i] = speedx; // * (1 - T) + speed_as5600[i] * T; // mm/s
        add_filament_meters(i, distance_E / 1000);
    }
    time_last = time_now;
}

enum filament_now_position_enum
{
    filament_idle,
    filament_sending_out,
    filament_using,
    filament_pulling_back,
    filament_redetect,
};
int filament_now_position[4];
bool wait = false;

bool Prepare_For_filament_Pull_Back(float_t OUT_filament_meters)
{
    bool wait = false;
    for (int i = 0; i < 4; i++)
    {
        if (filament_now_position[i] == filament_pulling_back)
        {
            // DEBUG_MY("last_total_distance: "); // Debug output
            // Debug_log_write_float(last_total_distance[i], 5);
            if (last_total_distance[i] < OUT_filament_meters)
            {
                // Unload when not reached
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pull, 100); // Drive motor to unload
                // Gradient light effect
                float npercent = (last_total_distance[i] / OUT_filament_meters) * 100.0f;
                MC_STU_RGB_set(i, 255 - ((255 / 100) * npercent), 125 - ((125 / 100) * npercent), (255 / 100) * npercent);
                // Unload not complete, needs priority handling
            }
            else
            {
                // Reached stop distance
                is_backing_out = false; // No need to continue recording distance
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100); // Stop motor
                filament_now_position[i] = filament_idle;               // Set current position to idle
                set_filament_motion(i, AMS_filament_motion::idle);      // Force idle state
                last_total_distance[i] = 0;                             // Reset unload distance
                // Unload complete
            }
            // Must wait as long as in unload state, until no longer unloading, next loop won't need to wait
            wait = true;
        }
    }
    return wait;
}
void motor_motion_switch() // Channel state switching function, only controls currently used channel, others set to stop
{
    int num = get_now_filament_num();                      // Current channel number
    uint16_t device_type = get_now_BambuBus_device_type(); // Device type
    for (int i = 0; i < 4; i++)
    {
        if (i != num)
        {
            filament_now_position[i] = filament_idle;
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 1000);
        }
        else if (MC_ONLINE_key_stu[num] == 1 || MC_ONLINE_key_stu[num] == 3) // Channel has filament
        {
            switch (get_filament_motion(num)) // Check simulator state
            {
            case AMS_filament_motion::need_send_out: // Need to feed
                MC_STU_RGB_set(num, 00, 255, 00);
                filament_now_position[num] = filament_sending_out;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_send, 100);
                break;
            case AMS_filament_motion::need_pull_back:
                pull_state_old = false; // Reset flag
                is_backing_out = true; // Mark as retracting
                filament_now_position[num] = filament_pulling_back;
                if (device_type == BambuBus_AMS_lite)
                {
                    MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pull, 100);
                }
                // Prepare_For_filament_Pull_Back(OUT_filament_meters); // Control unload completion by distance
                break;
            case AMS_filament_motion::before_pull_back:
            case AMS_filament_motion::on_use:
            {
                static uint64_t time_end = 0;
                uint64_t time_now = get_time64();
                if (filament_now_position[num] == filament_sending_out) // If channel just started feeding
                {
                    is_backing_out = false; // Set no need to record distance
                    pull_state_old = true; // Won't pull back first time, waits for low voltage trigger, prevents being pulled out right after entering
                    filament_now_position[num] = filament_using; // Mark as in use
                    time_end = time_now + 1500;                  // Prevent ungrasped, continue for 1.5 seconds
                }
                else if (filament_now_position[num] == filament_using) // Already triggered and in use
                {
                    last_total_distance[i] = 0; // Reset unload distance
                    if (time_now > time_end)
                    {                                          // Over 1.5 seconds, enter channel use for continued feeding
                        MC_STU_RGB_set(num, 255, 255, 255); // White
                        MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_on_use, 20);
                    }
                    else
                    {                                                                  // Was filament just detected
                        MC_STU_RGB_set(num, 128, 192, 128);                         // Light green
                        MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_slow_send, 100); // First 1.5 seconds slow feed, assist tool head grip
                    }
                }
                break;
            }
            case AMS_filament_motion::idle:
                filament_now_position[num] = filament_idle;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100);
                for (int i = 0; i < 4; i++)
                {
                    // Hardware normal
                    if (MC_ONLINE_key_stu[i] == 1 || MC_ONLINE_key_stu[i] == 0)
                    {   // Both triggered or no filament
                        MC_STU_RGB_set(i, 0, 0, 255); // Blue
                    }
                    else if (MC_ONLINE_key_stu[i] == 2)
                    {   // Only outer triggered
                        MC_STU_RGB_set(i, 255, 144, 0); // Orange / gold-like
                    }
                    else if (MC_ONLINE_key_stu[i] == 3)
                    {   // Only inner triggered
                        MC_STU_RGB_set(i, 0, 255, 255); // Cyan
                    }
                }
                break;
            }
        }
        else if (MC_ONLINE_key_stu[num] == 0) // 0: definitely no filament, 1: both triggered definitely has filament, 2: only outer triggered, 3: only inner triggered, has anti-disconnect feature
        {
            filament_now_position[num] = filament_idle;
            MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100);
            // MC_STU_RGB_set(num, 0, 0, 255);
        }
    }
}
// Schedule motors based on AMS simulator information
void motor_motion_run(int error)
{
    uint64_t time_now = get_time64();
    static uint64_t time_last = 0;
    float time_E = time_now - time_last; // Get time difference (ms)
    time_E = time_E / 1000;              // Convert to seconds
    uint16_t device_type = get_now_BambuBus_device_type();
    if (!error) // Normal mode
    {
        // Execute different motor control logic based on device type
        if (device_type == BambuBus_AMS_lite)
        {
            motor_motion_switch(); // Schedule motors
        }
        else if (device_type == BambuBus_AMS)
        {
            if (!Prepare_For_filament_Pull_Back(P1X_OUT_filament_meters)) // Negated (returns true) means no need to prioritize unloading, continue scheduling motors
            {
                motor_motion_switch(); // Schedule motors
            }
        }
    }
    else // Error mode
    {
        for (int i = 0; i < 4; i++)
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100); // Turn off motors
    }

    for (int i = 0; i < 4; i++)
    {
        /*if (!get_filament_online(i)) // If channel offline, motor not allowed to work
            MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);*/
        MOTOR_CONTROL[i].run(time_E); // Drive motor based on state info

        if (MC_PULL_stu[i] == 1)
        {
            MC_PULL_ONLINE_RGB_set(i, 255, 0, 0); // Pressure too high, red light
        }
        else if (MC_PULL_stu[i] == 0)
        { // Normal pressure
            if (MC_ONLINE_key_stu[i] == 1)
            { // Online and both micro-switches triggered
                int filament_colors_R = channel_colors[i][0];
                int filament_colors_G = channel_colors[i][1];
                int filament_colors_B = channel_colors[i][2];
                // Based on stored filament color
                MC_PULL_ONLINE_RGB_set(i, filament_colors_R, filament_colors_G, filament_colors_B);
                // White light
                // MC_STU_RGB_set(i, 255, 255, 255);
            }
            else
            {
                MC_PULL_ONLINE_RGB_set(i, 0, 0, 0); // No filament, no light
            }
        }
        else if (MC_PULL_stu[i] == -1)
        {
            MC_PULL_ONLINE_RGB_set(i, 0, 0, 255); // Pressure too low, blue light
        }
    }
    time_last = time_now;
}
// Motion control function
void Motion_control_run(int error)
{
    MC_PULL_ONLINE_read();

    AS5600_distance_updata();
    for (int i = 0; i < 4; i++)
    {
        if (MC_ONLINE_key_stu[i] == 0) {
            set_filament_online(i, false);
        } else if (MC_ONLINE_key_stu[i] == 1) {
            set_filament_online(i, true);
        } else if (MC_ONLINE_key_stu[i] == 3 && filament_now_position[i] == filament_using) {
            // If only inner triggered and in use, don't go offline yet
            set_filament_online(i, true);
        } else if (filament_now_position[i] == filament_redetect || (filament_now_position[i] == filament_pulling_back)) {
            // If in unload return or unloading, don't go offline yet
            set_filament_online(i, true);
        } else {
            set_filament_online(i, false);
        }
    }
    /*
        If outer micro-switch triggered, orange / gold-like
        If only inner micro-switch triggered, // cyan
        If both triggered, idle = blue, also means filament online, blue + white/channel saved color
    */

    if (error) // Error != 0
    {
        for (int i = 0; i < 4; i++)
        {
            set_filament_online(i, false);
            // filament_channel_inserted[i] = true; // For testing
            if (MC_ONLINE_key_stu[i] == 1)
            {                                        // Both triggered
                MC_STU_RGB_set(i, 0, 0, 255); // Blue
            }
            else if (MC_ONLINE_key_stu[i] == 2)
            {                                        // Only outer triggered
                MC_STU_RGB_set(i, 255, 144, 0); // Orange / gold-like
            }
            else if (MC_ONLINE_key_stu[i] == 3)
            {                                        // Only inner triggered
                MC_STU_RGB_set(i, 0, 255, 255); // Cyan
            } else if (MC_ONLINE_key_stu[i] == 0)
            {   // Not connected to printer and no filament
                MC_STU_RGB_set(i, 0, 0, 0); // Black
            }
        }
    } else { // Normally connected to printer
        // Setting colors here would cause repeated modifications
        for (int i = 0; i < 4; i++)
        {
            if ((MC_AS5600.online[i] == false) || (MC_AS5600.magnet_stu[i] == -1)) // AS5600 error
            {
                set_filament_online(i, false);
                MC_STU_ERROR[i] = true;
            }
        }
    }
    motor_motion_run(error);
}
// Set up PWM motor driver
void MC_PWM_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // Enable alternate function clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // Enable TIM2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // Enable TIM3 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // Enable TIM4 clock

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // Timer base configuration
    TIM_TimeBaseStructure.TIM_Period = 999;  // Period (x+1)
    TIM_TimeBaseStructure.TIM_Prescaler = 1; // Prescaler (x+1)
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM mode configuration
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // Duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure); // PA15
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // PB3
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // PB4
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); // PB5
    TIM_OC1Init(TIM4, &TIM_OCInitStructure); // PB6
    TIM_OC2Init(TIM4, &TIM_OCInitStructure); // PB7
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // PB8
    TIM_OC4Init(TIM4, &TIM_OCInitStructure); // PB9

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);    // TIM2 full remap - CH1-PA15/CH2-PB3
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // TIM3 partial remap - CH1-PB4/CH2-PB5
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);       // TIM4 no remap - CH1-PB6/CH2-PB7/CH3-PB8/CH4-PB9

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}
// Get PWM friction zero point (deprecated, assumed to be 50% duty cycle)
void MOTOR_get_pwm_zero()
{
    float pwm_zero[4] = {0, 0, 0, 0};
    MC_AS5600.updata_angle();

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];
    }
    for (int pwm = 300; pwm < 1000; pwm += 10)
    {
        MC_AS5600.updata_angle();
        for (int index = 0; index < 4; index++)
        {

            if (pwm_zero[index] == 0)
            {
                if (abs(MC_AS5600.raw_angle[index] - last_angle[index]) > 50)
                {
                    pwm_zero[index] = pwm;
                    pwm_zero[index] *= 0.90;
                    Motion_control_set_PWM(index, 0);
                }
                else if ((MC_AS5600.online[index] == true))
                {
                    Motion_control_set_PWM(index, -pwm);
                }
            }
            else
            {
                Motion_control_set_PWM(index, 0);
            }
        }
        delay(100);
    }
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(pwm_zero[index]);
    }
}
// Convert angle value to angle difference value
int M5600_angle_dis(int16_t angle1, int16_t angle2)
{

    int cir_E = angle1 - angle2;
    if ((angle1 > 3072) && (angle2 <= 1024))
    {
        cir_E = -4096;
    }
    else if ((angle1 <= 1024) && (angle2 > 3072))
    {
        cir_E = 4096;
    }
    return cir_E;
}

// Test motor rotation direction
void MOTOR_get_dir()
{
    int dir[4] = {0, 0, 0, 0};
    bool done = false;
    bool have_data = Motion_control_read();
    if (!have_data)
    {
        for (int index = 0; index < 4; index++)
        {
            Motion_control_data_save.Motion_control_dir[index] = 0;
        }
    }
    MC_AS5600.updata_angle(); // Read AS5600 initial angle value

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];                  // Record initial angle value
        dir[index] = Motion_control_data_save.Motion_control_dir[index]; // Record dir data from flash
    }
    //bool need_test = false; // Whether detection is needed
    bool need_save = false; // Whether state update is needed
    for (int index = 0; index < 4; index++)
    {
        if ((MC_AS5600.online[index] == true)) // Has AS5600, channel is online
        {
            if (Motion_control_data_save.Motion_control_dir[index] == 0) // Previous test result was 0, needs testing
            {
                Motion_control_set_PWM(index, 1000); // Turn on motor
                //need_test = true;                    // Set needs testing
                need_save = true;                    // State updated
            }
        }
        else
        {
            dir[index] = 0;   // Channel offline, clear its direction data
            need_save = true; // State updated
        }
    }
    int i = 0;
    while (done == false)
    {
        done = true;

        delay(10);                // Check every 10ms
        MC_AS5600.updata_angle(); // Update angle data

        if (i++ > 200) // No response for over 2s
        {
            for (int index = 0; index < 4; index++)
            {
                Motion_control_set_PWM(index, 0);                       // Stop
                Motion_control_data_save.Motion_control_dir[index] = 0; // Set direction to 0
            }
            break; // Exit loop
        }
        for (int index = 0; index < 4; index++) // Iterate
        {
            if ((MC_AS5600.online[index] == true) && (Motion_control_data_save.Motion_control_dir[index] == 0)) // For new channels
            {
                int angle_dis = M5600_angle_dis(MC_AS5600.raw_angle[index], last_angle[index]);
                if (abs(angle_dis) > 163) // Moved over 1mm
                {
                    Motion_control_set_PWM(index, 0); // Stop
                    if (angle_dis > 0)                // Here AS5600 faces the magnet, opposite to back-mount direction
                    {
                        dir[index] = 1;
                    }
                    else
                    {
                        dir[index] = -1;
                    }
                }
                else
                {
                    done = false; // No movement, continue waiting
                }
            }
        }
    }
    for (int index = 0; index < 4; index++) // Iterate through four motors
    {
        Motion_control_data_save.Motion_control_dir[index] = dir[index]; // Copy data
    }
    if (need_save) // If data needs saving
    {
        Motion_control_save(); // Save data
    }
}
// Initialize motor
// Specify direction
int first_boot = 1; // 1 means first boot, for executing startup-only tasks
void set_motor_directions(int dir0, int dir1, int dir2, int dir3)
{
    Motion_control_data_save.Motion_control_dir[0] = dir0;
    Motion_control_data_save.Motion_control_dir[1] = dir1;
    Motion_control_data_save.Motion_control_dir[2] = dir2;
    Motion_control_data_save.Motion_control_dir[3] = dir3;

    Motion_control_save(); // Save to flash
}
void MOTOR_init()
{

    MC_PWM_init();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    MC_AS5600.init(AS5600_SCL, AS5600_SDA, 4);
    // MOTOR_get_pwm_zero();
    // Auto direction
    MOTOR_get_dir();

    // For fixed motor direction
    if (first_boot == 1)
    { // First boot
        // set_motor_directions(1 , 1 , 1 , 1 ); // 1 is forward, -1 is reverse
        first_boot = 0;
    }
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(500);
        MOTOR_CONTROL[index].dir = Motion_control_data_save.Motion_control_dir[index];
    }
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        as5600_distance_save[i] = MC_AS5600.raw_angle[i];
    }
}
extern void RGB_update();
void Motion_control_init() // Initialize all motion and sensors
{
    MC_PULL_ONLINE_init();
    MC_PULL_ONLINE_read();
    MOTOR_init();
    
    /*
    // This is a blocking DEBUG code section
    while (1)
    {
        delay(10);
        MC_PULL_ONLINE_read();

        for (int i = 0; i < 4; i++)
        {
            MOTOR_CONTROL[i].set_motion(filament_motion_pressure_ctrl_on_use, 100);
            if (!get_filament_online(i)) // If channel offline, motor not allowed to work
                MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);
            MOTOR_CONTROL[i].run(0); // Drive motor based on state info
        }
        char s[100];
        int n = sprintf(s, "%d\n", (int)(MC_PULL_stu_raw[3] * 1000));
        DEBUG_num(s, n);
    }*/

    for (int i = 0; i < 4; i++)
    {
        // if(MC_AS5600.online[i]) // Use AS5600 signal to determine if channel is inserted
        // {
        //     filament_channel_inserted[i]=true;
        // }
        // else
        // {
        //     filament_channel_inserted[i]=false;
        // }
        filament_now_position[i] = filament_idle; // Set channel initial state to idle
    }
}
