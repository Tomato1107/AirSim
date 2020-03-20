// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

/**
 * @brief Interface for MIT Aerospace Controls Lab Snap Stack autopilot
 * @author Parker Lusk <plusk@mit.edu>
 * @date 18 March 2020
 */

#ifndef msr_airlib_SnapStackDroneController_hpp
#define msr_airlib_SnapStackDroneController_hpp

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/Common.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"

#include "client.h"
#include "server.h"
#include "sensor-imu/sensor_datatypes.h"
#include "esc_interface/esc_datatypes.h"
#include "esc_interface/esc_interface.h"

namespace msr { namespace airlib {

class SnapStackApi : public MultirotorApiBase {

public:
    SnapStackApi(const MultiRotorParams* vehicle_params, const AirSimSettings::VehicleSetting* vehicle_setting)
        : vehicle_params_(vehicle_params)
    {
        readSettings(*vehicle_setting);

        //TODO: set below properly for better high speed safety
        safety_params_.vel_to_breaking_dist = safety_params_.min_breaking_dist = 0;
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

        // send sensor data
        // receive actuator commands
    }
    virtual bool isApiControlEnabled() const override
    {
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
        return false;
    }
    virtual void enableApiControl(bool) override
    {
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
    }
    virtual bool armDisarm(bool) override
    {
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
        return false;
    }
    virtual GeoPoint getHomeGeoPoint() const override
    {
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
        return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
    }
    virtual void getStatusMessages(std::vector<std::string>&) override
    {
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
    }

    virtual const SensorCollection& getSensors() const override
    {
        return vehicle_params_->getSensors();
    }

public: //MultirotorApiBase implementation
    virtual real_T getActuation(unsigned int rotor_index) const override
    {
        // TODO: index array of motor commands
        return 0.0;
    }
    virtual size_t getActuatorCount() const override
    {
        return vehicle_params_->getParams().rotor_count;
    }
    virtual void moveByRC(const RCData& rc_data) override
    {
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
    }
    virtual void setSimulatedGroundTruth(const Kinematics::State* kinematics, const Environment* environment) override
    {
        // TODO: something?
        // board_->setGroundTruthKinematics(kinematics);
        // estimator_->setGroundTruthKinematics(kinematics, environment);
    }
    virtual bool setRCData(const RCData& rc_data) override
    {
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
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
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
        return {};
        //return what we received last time through setRCData
        // return last_rcData_;
    }

    virtual GeoPoint getGpsLocation() const override
    {
        // TODO: is this important?
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
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
        // pick a number, 3 meters is probably safe
        // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
        return params_.takeoff.takeoff_z;
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
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
    }

    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) override
    {
        unused(pitch);
        unused(roll);
        unused(z);
        unused(yaw);
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
    }

    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
    {
        unused(vx);
        unused(vy);
        unused(vz);
        unused(yaw_mode);
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
    }

    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
    {
        unused(vx);
        unused(vy);
        unused(z);
        unused(yaw_mode);
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
    }

    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
    {
        unused(x);
        unused(y);
        unused(z);
        unused(yaw_mode);
        Utils::log("Not Implemented", Utils::kLogLevelInfo);
    }

    virtual const MultirotorApiParams& getMultirotorApiParams() const override
    {
        return safety_params_;
    }

    //*** End: MultirotorApiBase implementation ***//

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
        params_.default_vehicle_state = simple_flight::VehicleState::fromString(
            vehicle_setting.default_vehicle_state == "" ? "Armed" : vehicle_setting.default_vehicle_state);

        // remote_control_id_ = vehicle_setting.rc.remote_control_id;
        // params_.rc.allow_api_when_disconnected = vehicle_setting.rc.allow_api_when_disconnected;
        // params_.rc.allow_api_always = vehicle_setting.allow_api_always;
    }

private:
    const MultiRotorParams* vehicle_params_;

    // int remote_control_id_ = 0;
    simple_flight::Params params_;

    MultirotorApiParams safety_params_;
};

}} //namespace
#endif