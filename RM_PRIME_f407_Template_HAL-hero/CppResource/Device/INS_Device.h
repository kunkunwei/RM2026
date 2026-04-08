#ifndef F407_RM_TMPLATE_HAL_INS_DEVICE_H
#define F407_RM_TMPLATE_HAL_INS_DEVICE_H

#include "main_cpp.h"
#include "Simple_IIR3.h"

namespace Device{

    //处理IMU等惯性传感器数据的类
    class INS_Device{
    public:

        struct Vector3{
            union {
                struct {
                    float x;
                    float y;
                    float z;
                };
                struct {
                    float yaw;
                    float pitch;
                    float roll;
                };
                float data[3];
            };
        };

        struct Quaternion{
            union {
                struct {
                    float s;
                    float x;
                    float y;
                    float z;
                };
                float data[4];
            };
        };

        static INS_Device& getInstance();
        void Init();
        void Insert(Vector3 acc, Vector3 gyro, Vector3 meg ,float dt);
        Vector3 getAngle();
        Vector3 getOmiga();
        Vector3 getAccel();

    private:
        INS_Device()= default;
        ~INS_Device()= default;
        INS_Device(const INS_Device&)= delete;

        Quaternion angle_quat = {0.0f, 0.0f, 0.0f, 0.0f};
        Vector3 angle = {0.0f, 0.0f, 0.0f};
        Vector3 omiga = {0.0f, 0.0f, 0.0f};

        Vector3 accel_tmp = {0.0f, 0.0f, 0.0f};
        Vector3 mag_tmp = {0.0f, 0.0f, 0.0f};


        constexpr static const float Filter_num[][4] = {{0.002329458745586203f, 0.f, 0.f, 0.f},{1.f, -1.929454039488895f, 0.93178349823448126f, 0.f}};
        Component::Simple_IIR3 acc_filter[3] = {
                Component::Simple_IIR3(Filter_num[0],Filter_num[1]),
                Component::Simple_IIR3(Filter_num[0],Filter_num[1]),
                Component::Simple_IIR3(Filter_num[0],Filter_num[1]),
        };

    };
}

#endif
