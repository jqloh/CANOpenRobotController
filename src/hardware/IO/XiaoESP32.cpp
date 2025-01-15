#include "Xiaoesp32.h"

#define INT16_MAX 32767

XiaoESP::XiaoESP(int responseID1_, int responseID2_, int responseID3_, int responseID4_) {
    spdlog::info("Robotous Sensor Created");

    // Change the parameters
    responseID1 = responseID1_;
    responseID2 = responseID2_;
    responseID3 = responseID3_;
    responseID4 = responseID4_;

    // Initialise variables as zeros
    accl_A = Eigen::VectorXd::Zero(3);
    accl_grav_A = Eigen::VectorXd::Zero(3);
    orient_A = Eigen::VectorXd::Zero(3);
    quat_A = Eigen::VectorXd::Zero(4);
}
XiaoESP::~XiaoESP() {
    /* restore the terminal settings */
    spdlog::debug("Xiao object deleted, echo enabled");
};
bool XiaoESP::configureMasterPDOs() {

    //spdlog::info("Teensy {} - RPDO {} Set", commandID, CO_setRPDO(&RPDOcommParaH, &RPDOMapParamH, RPDOCommEntryH, dataStoreRecordH, RPDOMapParamEntryH));
    UNSIGNED16 dataSize[8] = {1,1,1,1,1,1,1,1};
    void *dataEntryAccl_A[8] = {(void *)&rawData_A[0],
                            (void *)&rawData_A[1],
                            (void *)&rawData_A[2],
                            (void *)&rawData_A[3],
                            (void *)&rawData_A[4],
                            (void *)&rawData_A[5],
                            (void *)&rawData_A[6],
                            (void *)&rawData_A[7]};
    void *dataEntryAcclGravA[8] = {(void *)&rawData_A[8],
                          (void *)&rawData_A[9],
                          (void *)&rawData_A[10],
                          (void *)&rawData_A[11],
                          (void *)&rawData_A[12],
                          (void *)&rawData_A[13],
                          (void *)&rawData_A[14],
                          (void *)&rawData_A[15]};
    void *dataEntryOrient_A[8] = {(void *)&rawData_A[16],
                          (void *)&rawData_A[17],
                          (void *)&rawData_A[18],
                          (void *)&rawData_A[19],
                          (void *)&rawData_A[20],
                          (void *)&rawData_A[21],
                          (void *)&rawData_A[22],
                          (void *)&rawData_A[23]};
    void *dataEntryQuat_A[8] = {(void *)&rawData_A[24],
                          (void *)&rawData_A[25],
                          (void *)&rawData_A[26],
                          (void *)&rawData_A[27],
                          (void *)&rawData_A[28],
                          (void *)&rawData_A[29],
                          (void *)&rawData_A[30],
                          (void *)&rawData_A[31]};
    rpdo1 = new RPDO(responseID1, 0xff, dataEntryAccl_A, dataSize, lengthData);
    rpdo2 = new RPDO(responseID2, 0xff, dataEntryAcclGravA, dataSize, lengthData);
    rpdo3 = new RPDO(responseID3, 0xff, dataEntryOrient_A, dataSize, lengthData);
    rpdo4 = new RPDO(responseID4, 0xff, dataEntryQuat_A, dataSize, lengthData);

    spdlog::debug("TPOD and RPDO of XiaoESP initialised");
    return true;
}

float XiaoESP::range_mapping(float msg_val, float sensor_range, float msg_max) {
    return (msg_val / (msg_max /  sensor_range));
    //return msg_val / 100.;
}

void XiaoESP::updateInput() {
    // If the last command was a streamed command, update the local copy of forces

    int16_t accl_A_x = rawData_A[0] * 256 + rawData_A[1];
    int16_t accl_A_y = rawData_A[2] * 256 + rawData_A[3];
    int16_t accl_A_z = rawData_A[4] * 256 + rawData_A[5];
    int16_t accl_grav_A_x = rawData_A[8] * 256 + rawData_A[9];
    int16_t accl_grav_A_y = rawData_A[10] * 256 + rawData_A[11];
    int16_t accl_grav_A_z = rawData_A[12] * 256 + rawData_A[13];
    int16_t orient_A_yaw = rawData_A[16] * 256 + rawData_A[17];
    int16_t orient_A_pitch = rawData_A[18] * 256 + rawData_A[19];
    int16_t orient_A_roll = rawData_A[20] * 256 + rawData_A[21];
    int16_t quat_A_w = rawData_A[24] * 256 + rawData_A[25];
    int16_t quat_A_x = rawData_A[26] * 256 + rawData_A[27];
    int16_t quat_A_y = rawData_A[28] * 256 + rawData_A[29];
    int16_t quat_A_z = rawData_A[30] * 256 + rawData_A[31];

    accl_A[0] = range_mapping(accl_A_x, accl_range, INT16_MAX) ;
    accl_A[1] = range_mapping(accl_A_y, accl_range, INT16_MAX);
    accl_A[2] = range_mapping(accl_A_z, accl_range, INT16_MAX);
    accl_grav_A[0] = range_mapping(accl_grav_A_x, accl_range, INT16_MAX);
    accl_grav_A[1] = range_mapping(accl_grav_A_y, accl_range, INT16_MAX);
    accl_grav_A[2] = range_mapping(accl_grav_A_z, accl_range, INT16_MAX);
    orient_A[0] = range_mapping(orient_A_yaw, oren_yaw_range, INT16_MAX);
    orient_A[1] = range_mapping(orient_A_pitch, oren_pitch_range, INT16_MAX);
    orient_A[2] = range_mapping(orient_A_roll, oren_roll_range, INT16_MAX);
    quat_A[0] = range_mapping(quat_A_w,oren_quat_range,INT16_MAX);
    quat_A[1] = range_mapping(quat_A_x,oren_quat_range,INT16_MAX);
    quat_A[2] = range_mapping(quat_A_y,oren_quat_range,INT16_MAX);
    quat_A[3] = range_mapping(quat_A_z,oren_quat_range,INT16_MAX);

    // This is not that efficient if data is stopped. This could be modified to do a check first
    /*
    UNSIGNED16 Fx = rawData_A[1] * 256 + rawData_A[2];
    UNSIGNED16 Fy = rawData_A[3] * 256 + rawData_A[4];
    UNSIGNED16 Fz = rawData_A[5] * 256 + rawData_A[6];
    UNSIGNED16 Tx = rawData_A[7] * 256 + rawData_A[8];
    UNSIGNED16 Ty = rawData_A[9] * 256 + rawData_A[10];
    UNSIGNED16 Tz = rawData_A[11] * 256 + rawData_A[12];

    accl_A[0] = static_cast<INTEGER16> (Fx)/50.0;
    accl_A[1] = static_cast<INTEGER16>(Fy) / 50.0;
    accl_A[2] = static_cast<INTEGER16>(Fz) / 50.0;

    gyro_A[0] = static_cast<INTEGER16> (Tx)/2000.0 ;
    gyro_A[1] = static_cast<INTEGER16>(Ty) / 2000.0 ;
    gyro_A[2] = static_cast<INTEGER16>(Tz) / 2000.0 ;
    */
    // Else, don't do anything
}

Eigen::VectorXd& XiaoESP::getAccl() {
    return accl_A;
}

Eigen::VectorXd& XiaoESP::getAcclGrav() {
    return accl_grav_A;
}

Eigen::VectorXd& XiaoESP::getOrient() {
    return orient_A;
}

Eigen::VectorXd& XiaoESP::getQuat() {
    return quat_A;
}