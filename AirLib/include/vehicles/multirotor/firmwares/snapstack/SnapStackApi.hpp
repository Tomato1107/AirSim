// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

/**
 * @brief Interface for MIT Aerospace Controls Lab Snap Stack autopilot
 * @author Parker Lusk <plusk@mit.edu>
 * @date 18 March 2020
 */

#ifndef msr_airlib_SnapStackDroneController_hpp
#define msr_airlib_SnapStackDroneController_hpp

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"

// Sensors
#include "sensors/imu/ImuBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/lidar/LidarBase.hpp"

#include "client.h"
#include "server.h"
#include "ipc_common.h"
#include "sensor-imu/sensor_datatypes.h"
#include "esc_interface/esc_datatypes.h"
#include "esc_interface/esc_interface.h"

namespace msr { namespace airlib {

class SnapStackApi : public MultirotorApiBase {

public:
    SnapStackApi(const MultiRotorParams* vehicle_params, const AirSimSettings::VehicleSetting* vehicle_setting)
        : vehicle_params_(vehicle_params), first_imu_time_(0), escthread_stop_(false)
    {
        readSettings(*vehicle_setting);

        connect();

        //TODO: set below properly for better high speed safety
        safety_params_.vel_to_breaking_dist = safety_params_.min_breaking_dist = 0;
    }

    ~SnapStackApi()
    {
        escthread_stop_ = true;
        if (escthread_.joinable()) escthread_.join();
    }


public: //VehicleApiBase implementation
    virtual void resetImplementation() override
    {
        MultirotorApiBase::resetImplementation();

        // reset state
    }
    virtual void update() override
    {
        MultirotorApiBase::update();

        sendSensors();
        // receive actuator commands (done asynchronously)
    }
    virtual bool isApiControlEnabled() const override
    {
        Utils::log("isApiControlEnabled Not Implemented", Utils::kLogLevelInfo);
        return false;
    }
    virtual void enableApiControl(bool) override
    {
        Utils::log("enableApiControl Not Implemented", Utils::kLogLevelInfo);
    }
    virtual bool armDisarm(bool) override
    {
        Utils::log("armDisarm Not Implemented", Utils::kLogLevelInfo);
        return false;
    }
    virtual GeoPoint getHomeGeoPoint() const override
    {
        Utils::log("getHomeGeoPoint Not Implemented", Utils::kLogLevelInfo);
        return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
    }
    virtual void getStatusMessages(std::vector<std::string>&) override
    {
        // Utils::log("getStatusMessages Not Implemented", Utils::kLogLevelInfo);
    }

    virtual const SensorCollection& getSensors() const override
    {
        return vehicle_params_->getSensors();
    }

public: //MultirotorApiBase implementation
    virtual real_T getActuation(unsigned int rotor_index) const override
    {
        return motorcmds_[rotor_index];
    }
    virtual size_t getActuatorCount() const override
    {
        return vehicle_params_->getParams().rotor_count;
    }
    virtual void moveByRC(const RCData& rc_data) override
    {
        Utils::log("moveByRC Not Implemented", Utils::kLogLevelInfo);
    }
    virtual void setSimulatedGroundTruth(const Kinematics::State* kinematics, const Environment* environment) override
    {
        // TODO: something?
        // board_->setGroundTruthKinematics(kinematics);
        // estimator_->setGroundTruthKinematics(kinematics, environment);
    }
    virtual bool setRCData(const RCData& rc_data) override
    {
        // Utils::log("setRCData Not Implemented", Utils::kLogLevelInfo);
        return false;
    }

protected:
    virtual Kinematics::State getKinematicsEstimated() const override
    {
        // TODO: something?
        return {};
        // return AirSimSimpleFlightCommon::toKinematicsState3r(firmware_->offboardApi().
        //     getStateEstimator().getKinematicsEstimated());
    }

    virtual Vector3r getPosition() const override
    {
        // TODO: something?
        return {};
        // const auto& val = firmware_->offboardApi().getStateEstimator().getPosition();
        // return AirSimSimpleFlightCommon::toVector3r(val);
    }

    virtual Vector3r getVelocity() const override
    {
        // TODO: something?
        return {};
        // const auto& val = firmware_->offboardApi().getStateEstimator().getLinearVelocity();
        // return AirSimSimpleFlightCommon::toVector3r(val);
    }

    virtual Quaternionr getOrientation() const override
    {
        // TODO: something?
        return {};
        // const auto& val = firmware_->offboardApi().getStateEstimator().getOrientation();
        // return AirSimSimpleFlightCommon::toQuaternion(val);
    }

    virtual LandedState getLandedState() const override
    {
        // TODO: is this important?
        return LandedState::Landed;
        // return firmware_->offboardApi().getLandedState() ? LandedState::Landed : LandedState::Flying;
    }

    virtual RCData getRCData() const override
    {
        // TODO: is this important?
        Utils::log("getRCData Not Implemented", Utils::kLogLevelInfo);
        return {};
        //return what we received last time through setRCData
        // return last_rcData_;
    }

    virtual GeoPoint getGpsLocation() const override
    {
        // TODO: is this important?
        Utils::log("getGpsLocation Not Implemented", Utils::kLogLevelInfo);
        return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
        // return AirSimSimpleFlightCommon::toGeoPoint(firmware_->offboardApi().getGeoPoint());
    }

    virtual float getCommandPeriod() const override
    {
        // TODO: is this important?
        return 1.0f / 50; //50hz
    }

    virtual float getTakeoffZ() const override
    {
        Utils::log("getTakeoffZ Not Implemented", Utils::kLogLevelInfo);
        return 0.0;
    }

    virtual float getDistanceAccuracy() const override
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
    }

    virtual void commandRollPitchThrottle(float pitch, float roll, float throttle, float yaw_rate) override
    {
        unused(pitch);
        unused(roll);
        unused(throttle);
        unused(yaw_rate);
        Utils::log("commandRollPitchThrottle Not Implemented", Utils::kLogLevelInfo);
    }

    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) override
    {
        unused(pitch);
        unused(roll);
        unused(z);
        unused(yaw);
        Utils::log("commandRollPitchZ Not Implemented", Utils::kLogLevelInfo);
    }

    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
    {
        unused(vx);
        unused(vy);
        unused(vz);
        unused(yaw_mode);
        Utils::log("commandVelocity Not Implemented", Utils::kLogLevelInfo);
    }

    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
    {
        unused(vx);
        unused(vy);
        unused(z);
        unused(yaw_mode);
        Utils::log("commandVelocityZ Not Implemented", Utils::kLogLevelInfo);
    }

    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
    {
        unused(x);
        unused(y);
        unused(z);
        unused(yaw_mode);
        Utils::log("commandPosition Not Implemented", Utils::kLogLevelInfo);
    }

    virtual const MultirotorApiParams& getMultirotorApiParams() const override
    {
        return safety_params_;
    }

    //*** End: MultirotorApiBase implementation ***//

protected:
    void connect()
    {
        //
        // SIL Communications
        //

        const size_t imukey = acl::ipc::createKeyFromStr(vehicle_name_, "imu");
        const size_t esckey = acl::ipc::createKeyFromStr(vehicle_name_, "esc");

        // unique key is used to access the same shmem location
        imuserver_.reset(new acl::ipc::Server<sensor_imu>(imukey));
        escclient_.reset(new acl::ipc::Client<esc_commands>(esckey));

        //
        // ESC Commands thread
        //

        escthread_ = std::thread(&SnapStackApi::escReadThread, this);
    }

    const GpsBase* getGps() const
    {
        return static_cast<const GpsBase*>(getSensors().getByType(SensorBase::SensorType::Gps));
    }
    const ImuBase* getImu() const
    {
        return static_cast<const ImuBase*>(getSensors().getByType(SensorBase::SensorType::Imu));
    }
    const MagnetometerBase* getMagnetometer() const
    {
        return static_cast<const MagnetometerBase*>(getSensors().getByType(SensorBase::SensorType::Magnetometer));
    }
    const BarometerBase* getBarometer() const
    {
        return static_cast<const BarometerBase*>(getSensors().getByType(SensorBase::SensorType::Barometer));
    }
    const LidarBase* getLidar() const
    {
        return static_cast<const LidarBase*>(getSensors().getByType(SensorBase::SensorType::Lidar));
    }

private:
    //convert pitch, roll, yaw from -1 to 1 to PWM
    static uint16_t angleToPwm(float angle)
    {
        return static_cast<uint16_t>(angle * 500.0f + 1500.0f);
    }
    static uint16_t thrustToPwm(float thrust)
    {
        return static_cast<uint16_t>((thrust < 0 ? 0 : thrust) * 1000.0f + 1000.0f);
    }
    static uint16_t switchTopwm(float switchVal, uint maxSwitchVal = 1)
    {
        return static_cast<uint16_t>(1000.0f * switchVal / maxSwitchVal + 1000.0f);
    }

    void readSettings(const AirSimSettings::VehicleSetting& vehicle_setting)
    {
        vehicle_name_ = vehicle_setting.vehicle_name;
    }

    void sendSensors()
    {
        const auto& imu_output = getImu()->getOutput();

        imu_.timestamp_in_us = static_cast<uint64_t>(ClockFactory::get()->nowNanos()*1.0e-3);
        imu_.sequence_number++;
        // the orientation of the IMU corresponds to the sf board (eagle8074). AirSim is NED.
        imu_.linear_acceleration[0] = imu_output.linear_acceleration[0] / EarthUtils::Gravity;
        imu_.linear_acceleration[1] = imu_output.linear_acceleration[1] / EarthUtils::Gravity;
        imu_.linear_acceleration[2] = imu_output.linear_acceleration[2] / EarthUtils::Gravity;
        imu_.angular_velocity[0] = imu_output.angular_velocity[0];
        imu_.angular_velocity[1] = imu_output.angular_velocity[1];
        imu_.angular_velocity[2] = imu_output.angular_velocity[2];

        // Wait for imu to settle before sending
        constexpr uint64_t IMU_SETTLE = 500e3;
        if (first_imu_time_ == 0) first_imu_time_ = imu_.timestamp_in_us;
        if (imu_.timestamp_in_us - first_imu_time_ < IMU_SETTLE) return;

        imuserver_->send(imu_);
    }

    void escReadThread()
    {
        static constexpr uint16_t PWM_MAX = acl::ESCInterface::PWM_MAX_PULSE_WIDTH;
        static constexpr uint16_t PWM_MIN = acl::ESCInterface::PWM_MIN_PULSE_WIDTH;

        while (!escthread_stop_) {
            esc_commands esccmds;
            bool rcvd = escclient_->read(&esccmds);

            if (rcvd) {
                std::lock_guard<std::mutex> lck(escmtx_);
                motorcmds_[0] = (esccmds.pwm[0] - PWM_MIN) / static_cast<double>(PWM_MAX - PWM_MIN);
                motorcmds_[1] = (esccmds.pwm[1] - PWM_MIN) / static_cast<double>(PWM_MAX - PWM_MIN);
                motorcmds_[2] = (esccmds.pwm[2] - PWM_MIN) / static_cast<double>(PWM_MAX - PWM_MIN);
                motorcmds_[3] = (esccmds.pwm[3] - PWM_MIN) / static_cast<double>(PWM_MAX - PWM_MIN);
            }
        }
    }

private:
    const MultiRotorParams* vehicle_params_;

    std::string vehicle_name_;

    /// |brief IMU sensor data server
    std::unique_ptr<acl::ipc::Server<sensor_imu>> imuserver_;
    uint64_t first_imu_time_;
    sensor_imu imu_;

    /// \brief Thread for reading motor commands
    std::unique_ptr<acl::ipc::Client<esc_commands>> escclient_;
    std::atomic<bool> escthread_stop_;
    static constexpr int MOTORS = 4;
    float motorcmds_[MOTORS] = { 0.0f };
    std::thread escthread_;
    std::mutex escmtx_;

    MultirotorApiParams safety_params_;
};

}} //namespace
#endif