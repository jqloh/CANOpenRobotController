/*/**
 * \file IMUXiao.h
 * \author JQ Loh , Ming Rui Sun
 * \brief  Class representing IMU measurements from BNO055 - Seeeduino XIAO. 
 * 
 *    NOTE: this is not a CANOpen Device, and the PDO-like messages are send on non-standard COB-IDs
 * 
 * \version 0.1
 * \date 2024-05-20
 * \copyright Copyright (c) 2024
 *
 */

#ifndef IMUXIAO_H_INCLUDED
#define IMUXIAO_H_INCLUDED

#include <CANopen.h>
#include <CO_command.h>
#include <string.h>

#include <Eigen/Dense>

#include "InputDevice.h"
#include "logging.h"
#include "RPDO.h"
#include "TPDO.h"

class IMUXiao : public InputDevice {
   private:
        bool initialised;
        int commandID;     //   COB-ID of command messages    
        int responseID1; // COB-ID of 1st received message
        int responseID2;  // COB-ID of 2nd received message
        int responseID3;  // COB-ID of 2nd received message
        int responseID4; // COB-ID of 1st received message

        bool streaming=false; 

        // Objects representing the PDOs (used to create the PDOs in the OD)
        RPDO *rpdo1;
        RPDO *rpdo2;
        RPDO *rpdo3;
        RPDO *rpdo4;

        /// Raw data - these variables are linked to the PDOs
        UNSIGNED8 rawIMUData[32] = {0};
        UNSIGNED8 cmdData = 0;
        UNSIGNED32 cmdDataPad = 0; // This is to make sure that the message is the full 8 bytes because of Robotous' not-CANopen implementation

        // Number of mapped parameters for RPDOs (lengthData) and TPDO (lengthCmd)
        UNSIGNED8 lengthData =8; // 8 for each of the RPDOs - I cheat and reuse this variable
        UNSIGNED8 lengthCmd = 2; // Second one is for padding

        // BNO 9-DOF measurement ranges
        float accl_range = 100.;
        float gyro_range = 2000.;
        float orien_roll_range = 180.;
        float orien_pitch_range = 90.;
        float orien_yaw_range = 360.;
        float orien_quat_range = 1.;

        // OD Parameters
        // Will need to be modified to take into number of items, data size and location
        // Data size and number of items will be constant, function will be used to change location
        // Data variables
        Eigen::VectorXd accl;
        Eigen::VectorXd accl_grav;
        Eigen::VectorXd orient;
        Eigen::VectorXd quat;

        /**
         * \brief map message to actual sensor readings
         * 
         */
        float range_mapping(float msg_val, float sensor_range, float msg_max);

       public:
        /**
        * \brief Sets up the IMU object, including data storage and setting up PDOs
        *
        * \param commandID_ the COB-ID used to send messages to this device
        * \param responseID1_ the COB-ID of the first data message (sent from this device) 
        * \param responseID2_ the COB-ID of the second data message (sent from this device) 
        */
        IMUXiao(int responseID1_, int responseID2_, int responseID3_, int responseID4_);
        ~IMUXiao();

        /**
         * \brief Sets up the receiving PDOs (note: will have issues if commands are sent, as the response are on the same COB-IDs)
         * 
         */
        bool configureMasterPDOs();

        /**
         * \brief Updates the IMU readings from the raw data
         * 
         */
        void updateInput();

        /**
         * \brief Get the acceleration object
         * 
         * \return Eigen::VectorXd X,Y,Z acceleration
         */
        Eigen::VectorXd& getAccl();

        /**
         * \brief Get the acceleration_grav object
         * 
         * \return Eigen::VectorXd 
         */
        Eigen::VectorXd& getAcclGrav();

        /**
         * \brief Get the orientation object
         * 
         * \return Eigen::VectorXd 
         */
        Eigen::VectorXd& getOrient();

        /**
         * \brief Get the quaternion object
         * 
         * \return Eigen::VectorXd 
         */
        Eigen::VectorXd& getQuat();


};
#endif