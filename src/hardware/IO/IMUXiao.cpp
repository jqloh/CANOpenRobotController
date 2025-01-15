#include "IMUXiao.h"

#define INT16_MAX 32767

IMUXiao::IMUXiao(int responseID1_, int responseID2_, int responseID3_, int responseID4_):initialised(false) {
    spdlog::info("IMU Sensor Created");

    // Change the parameters
    responseID1 = responseID1_;
    responseID2 = responseID2_;
    responseID3 = responseID3_;
    responseID4 = responseID4_;

    // Initialise variables as zeros
    accl = Eigen::VectorXd::Zero(3);
    accl_grav = Eigen::VectorXd::Zero(3);
    orient = Eigen::VectorXd::Zero(3);
    quat = Eigen::VectorXd::Zero(4);

    // TODO: to include initialization check in the future
    initialised = false;
}
IMUXiao::~IMUXiao() {
    spdlog::debug("IMU object deleted");
};
bool IMUXiao::configureMasterPDOs() {

    //spdlog::info("IMU_Xiao {} - RPDO {} Set", commandID, CO_setRPDO(&RPDOcommParaH, &RPDOMapParamH, RPDOCommEntryH, dataStoreRecordH, RPDOMapParamEntryH));
    UNSIGNED16 dataSize[8] = {1,1,1,1,1,1,1,1};
    void *dataEntryaccl[8] = {(void *)&rawIMUData[0],
                            (void *)&rawIMUData[1],
                            (void *)&rawIMUData[2],
                            (void *)&rawIMUData[3],
                            (void *)&rawIMUData[4],
                            (void *)&rawIMUData[5],
                            (void *)&rawIMUData[6],
                            (void *)&rawIMUData[7]};
    void *dataEntryAcclGrav[8] = {(void *)&rawIMUData[8],
                          (void *)&rawIMUData[9],
                          (void *)&rawIMUData[10],
                          (void *)&rawIMUData[11],
                          (void *)&rawIMUData[12],
                          (void *)&rawIMUData[13],
                          (void *)&rawIMUData[14],
                          (void *)&rawIMUData[15]};
    void *dataEntryorient[8] = {(void *)&rawIMUData[16],
                          (void *)&rawIMUData[17],
                          (void *)&rawIMUData[18],
                          (void *)&rawIMUData[19],
                          (void *)&rawIMUData[20],
                          (void *)&rawIMUData[21],
                          (void *)&rawIMUData[22],
                          (void *)&rawIMUData[23]};
    void *dataEntryquat[8] = {(void *)&rawIMUData[24],
                          (void *)&rawIMUData[25],
                          (void *)&rawIMUData[26],
                          (void *)&rawIMUData[27],
                          (void *)&rawIMUData[28],
                          (void *)&rawIMUData[29],
                          (void *)&rawIMUData[30],
                          (void *)&rawIMUData[31]};
    rpdo1 = new RPDO(responseID1, 0xff, dataEntryaccl, dataSize, lengthData);
    rpdo2 = new RPDO(responseID2, 0xff, dataEntryAcclGrav, dataSize, lengthData);
    rpdo3 = new RPDO(responseID3, 0xff, dataEntryorient, dataSize, lengthData);
    rpdo4 = new RPDO(responseID4, 0xff, dataEntryquat, dataSize, lengthData);

    spdlog::debug("TPOD and RPDO of IMUXiao initialised");
    return true;
}

float IMUXiao::range_mapping(float msg_val, float sensor_range, float msg_max) {
    return (msg_val / (msg_max /  sensor_range));
    //return msg_val / 100.;
}

void IMUXiao::updateInput() {
    // If the last command was a streamed command, update the local copy of forces

    int16_t accl_x = rawIMUData[0] * 256 + rawIMUData[1];
    int16_t accl_y = rawIMUData[2] * 256 + rawIMUData[3];
    int16_t accl_z = rawIMUData[4] * 256 + rawIMUData[5];
    int16_t accl_grav_x = rawIMUData[8] * 256 + rawIMUData[9];
    int16_t accl_grav_y = rawIMUData[10] * 256 + rawIMUData[11];
    int16_t accl_grav_z = rawIMUData[12] * 256 + rawIMUData[13];
    int16_t orient_yaw = rawIMUData[16] * 256 + rawIMUData[17];
    int16_t orient_pitch = rawIMUData[18] * 256 + rawIMUData[19];
    int16_t orient_roll = rawIMUData[20] * 256 + rawIMUData[21];
    int16_t quat_w = rawIMUData[24] * 256 + rawIMUData[25];
    int16_t quat_x = rawIMUData[26] * 256 + rawIMUData[27];
    int16_t quat_y = rawIMUData[28] * 256 + rawIMUData[29];
    int16_t quat_z = rawIMUData[30] * 256 + rawIMUData[31];

    accl[0] = range_mapping(accl_x, accl_range, INT16_MAX) ;
    accl[1] = range_mapping(accl_y, accl_range, INT16_MAX);
    accl[2] = range_mapping(accl_z, accl_range, INT16_MAX);
    accl_grav[0] = range_mapping(accl_grav_x, accl_range, INT16_MAX);
    accl_grav[1] = range_mapping(accl_grav_y, accl_range, INT16_MAX);
    accl_grav[2] = range_mapping(accl_grav_z, accl_range, INT16_MAX);
    orient[0] = range_mapping(orient_yaw, orien_yaw_range, INT16_MAX);
    orient[1] = range_mapping(orient_pitch, orien_pitch_range, INT16_MAX);
    orient[2] = range_mapping(orient_roll, orien_roll_range, INT16_MAX);
    quat[0] = range_mapping(quat_w,orien_quat_range,INT16_MAX);
    quat[1] = range_mapping(quat_x,orien_quat_range,INT16_MAX);
    quat[2] = range_mapping(quat_y,orien_quat_range,INT16_MAX);
    quat[3] = range_mapping(quat_z,orien_quat_range,INT16_MAX);
}

Eigen::VectorXd& IMUXiao::getAccl() {
    return accl;
}

Eigen::VectorXd& IMUXiao::getAcclGrav() {
    return accl_grav;
}

Eigen::VectorXd& IMUXiao::getOrient() {
    return orient;
}

Eigen::VectorXd& IMUXiao::getQuat() {
    return quat;
}