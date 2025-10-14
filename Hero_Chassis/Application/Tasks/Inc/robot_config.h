#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// 机器人物理参数
#define ROBOT_WHEEL_RADIUS          0.075f      // 轮子半径 (m)
#define ROBOT_CHASSIS_RADIUS        0.2f        // 底盘半径 (m)

// 运动学限制
typedef struct {
    struct {
        float max_speed_x;          // 最大前后速度 (m/s)
        float max_speed_y;          // 最大左右速度 (m/s)
        float max_speed_z;          // 最大旋转速度 (rad/s)
        float max_acceleration_x;    // 最大前后加速度 (m/s^2)
        float max_acceleration_y;    // 最大左右加速度 (m/s^2)
        float max_acceleration_z;    // 最大旋转加速度 (rad/s^2)
    } chassis;

    struct {
        float pitch_max_angle;      // pitch最大角度 (rad)
        float pitch_min_angle;      // pitch最小角度 (rad)
        float yaw_max_speed;        // yaw最大速度 (rad/s)
        float pitch_max_speed;      // pitch最大速度 (rad/s)
    } gimbal;
} robot_limits_t;

// 控制参数
typedef struct {
    struct {
        float speed_pid[4];         // 速度环PID参数 (kp, ki, kd, max_out)
        float follow_pid[4];        // 跟随PID参数
        float rotation_speed;        // 小陀螺旋转速度
    } chassis;

    struct {
        float yaw_speed_pid[4];     // yaw速度环PID
        float yaw_angle_pid[4];     // yaw角度环PID
        float pitch_speed_pid[4];   // pitch速度环PID
        float pitch_angle_pid[4];   // pitch角度环PID
    } gimbal;

    struct {
        float motor_speed_pid[4];   // 发射电机速度环PID
        float trigger_angle_pid[4]; // 拨弹电机角度环PID
    } shooter;
} control_params_t;

// 遥控器参数
typedef struct {
    float chassis_vx_rc_sens;      // 前后速度灵敏度
    float chassis_vy_rc_sens;      // 左右速度灵敏度
    float chassis_wz_rc_sens;      // 旋转速度灵敏度
    float gimbal_pitch_rc_sens;    // 云台pitch灵敏度
    float gimbal_yaw_rc_sens;      // 云台yaw灵敏度
    int16_t rc_deadband;           // 遥控器死区
} remote_params_t;

// 默认参数配置
extern const robot_limits_t DEFAULT_ROBOT_LIMITS;
extern const control_params_t DEFAULT_CONTROL_PARAMS;
extern const remote_params_t DEFAULT_REMOTE_PARAMS;

// 当前使用的参数配置
extern robot_limits_t g_robot_limits;
extern control_params_t g_control_params;
extern remote_params_t g_remote_params;

// 参数初始化函数
void robot_config_init(void);
void load_default_params(void);

#endif // ROBOT_CONFIG_H
