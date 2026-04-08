#include "INS_Device.h"
#include "AHRS.h"


using namespace Device;


INS_Device& INS_Device::getInstance(){
    static INS_Device instance;
    return instance;
}

void INS_Device::Init(){
    AHRS_init(angle_quat.data, accel_tmp.data, mag_tmp.data);

}

void INS_Device::Insert(Vector3 acc, Vector3 gyro, Vector3 meg ,float dt){

    for (int i = 0; i < 3; ++i) {
        accel_tmp.data[i] = acc_filter[i].processSample(acc.data[i]);
        mag_tmp.data[i] = meg.data[i];
        omiga.data[i] = gyro.data[i];
    }

    AHRS_update(angle_quat.data, dt, omiga.data, accel_tmp.data, mag_tmp.data);
    get_angle(angle_quat.data, &angle.yaw, &angle.roll, &angle.pitch);

    angle.pitch*= -1.f;

}

INS_Device::Vector3 INS_Device::getAngle(){
    return angle;
}

INS_Device::Vector3 INS_Device::getOmiga(){
    return omiga;
}
INS_Device::Vector3 INS_Device::getAccel(){
    return accel_tmp;
}