//
//  fr_standardization_flightcontroller_model.hpp
//  FlightRecordStandardization
//
//  Copyright © 2019 DJISDK. All rights reserved.
//

#ifndef fr_standardization_flightcontroller_model_hpp
#define fr_standardization_flightcontroller_model_hpp

#include <stdio.h>
#include "fr_standardization_common.hpp"
#include <string>
#include "fr_standardization_flight_controller_interface.h"

namespace DJIFR {
    namespace standardization {
    
        //MARK: - VirtualStickFlightControlDataImp
    
        class VirtualStickFlightControlDataImp : public VirtualStickFlightControlData {
        public:
            VirtualStickFlightControlDataImp();
            virtual ~VirtualStickFlightControlDataImp();
            
            /// Gets aircraft's velocity [m/s] along the y-axis or angle value [in degrees] for
            /// pitch. Use `DJIVirtualStickRollPitchControlMode` to set the velocity or angle
            /// mode. Note that the argument has different meanings in different coordinate
            /// systems. See the <i>Flight Controller User Guide</i> for more information.
            float pitch() const override;
            void set_pitch(float pitch);
            
            /// Gets aircraft's velocity [m/s] along the x-axis or angle value [in degrees] for
            /// roll. Use `DJIVirtualStickRollPitchControlMode` to set the velocity or angle
            /// mode. Note that the argument has different meanings in different coordinate
            /// systems. See the <i>Flight Controller User Guide</i> for more information.
            float roll() const override;
            void set_roll(float roll);
            
            /// Angular velocity [degrees/s] or angle [degrees] value for yaw. Use
            /// `DJIVirtualStickYawControlMode` to set angular velocity or angle mode.
            float yaw() const override;
            void set_yaw(float yaw);
            
            /// Gets aircraft's velocity [m/s] or altitude [m] value for verticalControl. Use
            /// `DJIVirtualStickYawControlMode` to set velocity or altitude mode.
            float verticalThrottle() const override;
            void set_verticalThrottle(float verticalThrottle);
            
            /// The vertical control mode for virtual stick. CAUTION: It will be reset to
            /// `DJIVirtualStickVerticalControlModeVelocity` when the flight controller is
            /// reconnected.
            VirtualStickVerticalControlMode verticalControlMode() const override;
            void set_verticalControlMode(VirtualStickVerticalControlMode verticalControlMode);
            
            /// The roll and pitch control mode for virtual stick. CAUTION: It will be reset to
            /// `DJIVirtualStickRollPitchControlModeAngle` when the flight controller is
            /// reconnected.
            VirtualStickRollPitchControlMode rollPitchControlMode() const override;
            void set_rollPitchControlMode(VirtualStickRollPitchControlMode rollPitchControlMode);
            
            /// The yaw control mode for virtual stick. CAUTION: It will be reset to
            /// `DJIVirtualStickYawControlModeAngle` when the flight controller is reconnected.
            VirtualStickYawControlMode yawControlMode() const override;
            void set_yawControlMode(VirtualStickYawControlMode yawControlMode);
            
            /// The coordinate system for virtual stick. CAUTION: It will be reset to
            /// `DJIVirtualStickFlightCoordinateSystemGround` when the flight controller is
            /// reconnected.
            VirtualStickFlightCoordinateSystem rollPitchCoordinateSystem() const override;
            void set_rollPitchCoordinateSystem(VirtualStickFlightCoordinateSystem rollPitchCoordinateSystem);
            
        private:
            float pitch_;
            
            float roll_;
            
            float yaw_;
            
            float verticalThrottle_;
            
            VirtualStickVerticalControlMode verticalControlMode_;
            
            VirtualStickRollPitchControlMode rollPitchControlMode_;
            
            VirtualStickYawControlMode yawControlMode_;
            
            VirtualStickFlightCoordinateSystem rollPitchCoordinateSystem_;
        };
    
        //MARK: - FlightControllerStateImp
    
        class FlightControllerStateImp : public FlightControllerState {
        public:
            FlightControllerStateImp();
            virtual ~FlightControllerStateImp();
            
            std::shared_ptr<FlightControllerStateImp> clone();
            
            /// The attitude of the aircraft, where the pitch, roll, and yaw values will be in
            /// the range of [-180, 180] degrees. If its pitch, roll, and yaw values are 0, the
            /// aircraft will  be hovering level with a True North heading.
            AttitudeSharedPtr attitude() const override;
            void set_attitude(const AttitudeSharedPtr& attitude);
            
            VelocitySharedPtr velocity() const override;
            void set_velocity(const VelocitySharedPtr& velocity);
            
            /// Home location of the aircraft as a coordinate.
            LocationCoordinate2DSharedPtr homeLocationCoordinate() const override;
            void set_homeLocationCoordinate(const LocationCoordinate2DSharedPtr& homeLocationCoordinate);
            
            /// Relative altitude of the aircraft home location relative to sea level, in meters.
            float takeoffLocationAltitude() const override;
            void set_takeoffLocationAltitude(float takeoffLocationAltitude);
            
            /// The current location of the aircraft as a coordinate. `nil` if the location is invalid.
            LocationCoordinate2DSharedPtr aircraftLocation() const override;
            void set_aircraftLocation(const LocationCoordinate2DSharedPtr& aircraftLocation);
            
            /// Relative altitude of the aircraft relative to take off location, measured by
            /// the barometer, in meters.
            float altitude() const override;
            void set_altitude(float altitude);

            /// altitude from sensor
            float vpsHeight() const override;
            void set_vpsHeight(float vpsHeight);
            
            /// Flight controller flight mode.
            FlightMode flightMode() const override;
            void set_flightMode(FlightMode flightMode);
            
            /// Whether the GPS is being used
            bool isGPSBeingUsed() const override;
            void set_isGPSBeingUsed(bool isGPSBeingUsed);
            
            /// Aircraft's current GPS signal quality.
            GPSSignalLevel gpsLevel() const override;
            void set_gpsLevel(GPSSignalLevel level);
            
            /// GPS satellite count.
            int32_t satelliteCount() const override;
            void set_satelliteCount(int32_t satelliteCount);
            
            /// The estimated remaining time, in seconds, it will take the aircraft to go home
            /// with a 10% battery buffer remaining. This time includes landing the aircraft. If
            /// the aircraft is using the simulator, this value will be 0.
            int32_t remainingFlightTime() const override;
            void set_remainingFlightTime(int32_t remainingFlightTime);
            
            /// The battery percentage, in the range of [0,100], needed for the aircraft to move
            /// downward from its current position and land.
            int32_t batteryPercentageNeededToLandFromCurrentHeight() const override;
            void set_batteryPercentageNeededToLandFromCurrentHeight(int32_t batteryPercentageNeededToLandFromCurrentHeight);
            
            /// The estimated battery percentage, in the range of [0,100], needed for the
            /// aircraft to go home and have 10% battery remaining. This includes landing of the aircraft.
            int32_t batteryPercentageNeededToGoHome() const override;
            void set_batteryPercentageNeededToGoHome(int32_t batteryPercentageNeededToGoHome);
            
            /// The Smart Return-To-Home (RTH) state for the current flight.
            SmartRTHState smartRTHState() const override;
            void set_smartRTHState(SmartRTHState smartRTHState);
            
            /// Sets the FailSafe action for when the connection between remote controller and aircraft is lost.
            ConnectionFailSafeBehavior behavior() const override;
            void set_behavior(ConnectionFailSafeBehavior behavior);
            
            /// The virtual stick commands
            VirtualStickFlightControlDataSharedPtr virtualControlData() const override;
            void set_virtualControlData(const VirtualStickFlightControlDataSharedPtr& virtualControlData);
            
            ///  `YES` if the signal is lost between remote controller and aircraft, and FailSafe is enabled.
            bool isFailsafeEnabled() const override;
            void set_isFailsafeEnabled(bool isFailsafeEnabled);
            
            /// `YES` if motors are on.
            bool areMotorsOn() const override;
            void set_areMotorsOn(bool areMotorsOn);
            
            /// `YES` if the home point has been set.
            bool isHomeLocationSet() const override;
            void set_isHomeLocationSet(bool isHomeLocationSet);
            
            /// `YES` if the clearance between the aircraft and the ground is less than 0.3m,
            /// and  confirmation from the user is needed to continue the landing. When the
            /// confirmation is needed, `confirmLandingWithCompletion` in `DJIFlightController`
            /// can be used to continue landing. It is
            /// supported by flight controller firmware 3.2.0.0 or above.
            bool isLandingConfirmationNeeded() const override;
            void set_isLandingConfirmationNeeded(bool isLandingConfirmationNeeded);
            
            /// `YES` if the aircraft has reached its flight height limit.
            bool hasReachedMaxFlightHeight() const override;
            void set_hasReachedMaxFlightHeight(bool hasReachedMaxFlightHeight);
            
            /// `YES` if the aircraft has reached its flight radius limit.
            bool hasReachedMaxFlightRadius() const override;
            void set_hasReachedMaxFlightRadius(bool hasReachedMaxFlightRadius);
            
            /// Warning related to high winds.
            FlightWindWarning windWarning() const override;
            void set_windWarning(FlightWindWarning windWarning);
            
            /// The count of flights within the battery life cycle. Cleared when power-on.
            int32_t countOfFlights() const override;
            void set_countOfFlights(int32_t countOfFlights);
            
            /// The current index of the flight log on the aircraft. It is useful to find the
            /// corresponding flight log.
            int32_t flightLogIndex() const override;
            void set_flightLogIndex(int32_t flightLogIndex);
            
            /// `YES` if aircraft is flying.
            bool isFlying() const override;
            void set_isFlying(bool isFlying);
            
            /// The accumulated flight time, in seconds, since the aircraft's motors were turned on.
            double flightTimeInSeconds() const override;
            void set_flightTimeInSeconds(double flightTimeInSeconds);
            
            /// Cumulative flight distance of the aircraft in meters
            double cumulativeFlightDistance() const override;
            void set_cumulativeFlightDistance(double cumulativeFlightDistance);
            
            /// The countdown [in seconds] for the Smart Return-To-Home [RTH]. Once the
            /// countdown reaches 0, the aircraft will  execute an automatic go-home procedure.
            /// It is only valid when  `smartRTHState`  is
            /// `DJIFlightControllerSmartRTHStateCountingDown`.
            int32_t smartRTHCountdown() const override;
            void set_smartRTHCountdown(int32_t smartRTHCountdown);
            
            uint8_t versionC() const;
            void set_versionC(uint8_t versionC);
            
            uint8_t drone_type() const;
            void set_drone_type(uint8_t drone_type);

            //custom
            int32_t flyCCommand() const;
            void set_flyCCommand(int flyCCommand);

            int32_t canIocWork() const;
            void set_canIocWork(int canIocWork);

            int32_t isSwaveWork() const;
            void set_isSwaveWork(int isSwaveWork);

            int32_t GoHomeStatus() const;
            void set_GoHomeStatus(int GoHomeStatus);

            int32_t isVisionUsed() const;
            void set_isVisionUsed(int isVisionUsed);

            int32_t batteryWarning() const;
            void set_batteryWarning(int batteryWarning);

            int32_t isImuPreheated() const;
            void set_isImuPreheated(int isImuPreheated);

            int32_t modeChannel() const;
            void set_modeChannel(int modeChannel);

            int32_t isCompassError() const;
            void set_isCompassError(int isCompassError);

            int32_t waveError() const;
            void set_waveError(int waveError);

            int32_t batteryType() const;
            void set_batteryType(int batteryType);

            int32_t isOutOfLimit() const;
            void set_isOutOfLimit(int isOutOfLimit);

            int32_t isGoHomeHeightModified() const;
            void set_isGoHomeHeightModified(int isGoHomeHeightModified);

            int32_t isPropellerCatapult() const;
            void set_isPropellerCatapult(int isPropellerCatapult);

            int32_t isMotorBlocked() const;
            void set_isMotorBlocked(int isMotorBlocked);

            int32_t isNotEnoughForce() const;
            void set_isNotEnoughForce(int isNotEnoughForce);

            int32_t isBarometerDeadInAir() const;
            void set_isBarometerDeadInAir(int isBarometerDeadInAir);

            int32_t isVibrating() const;
            void set_isVibrating(int isVibrating);

            int32_t isAcceletorOverRange() const;
            void set_isAcceletorOverRange(int isAcceletorOverRange);

            int32_t flighAction() const;
            void set_flighAction(int flighAction);

            int32_t motorStartFailedCause() const;
            void set_motorStartFailedCause(int motorStartFailedCause);

            int32_t nonGpsCause() const;
            void set_nonGpsCause(int nonGpsCause);

            int32_t waypointLimitMode() const;
            void set_waypointLimitMode(int waypointLimitMode);

            int32_t battery() const;
            void set_battery(int battery);

            int32_t motorRevolution() const;
            void set_motorRevolution(int motorRevolution);

            int32_t ImuInitFailReason() const;
            void set_ImuInitFailReason(int ImuInitFailReason);

            int32_t ctrlDevice() const;
            void set_ctrlDevice(int ctrlDevice);

            
        private:
            AttitudeSharedPtr attitude_;
            
            VelocitySharedPtr velocity_;
            
            LocationCoordinate2DSharedPtr homeLocationCoordinate_;
            
            float takeoffLocationAltitude_;
            
            LocationCoordinate2DSharedPtr aircraftLocation_;
            
            float altitude_;

            float vpsHeight_;
            
            FlightMode flightMode_;
            
            bool isGPSBeingUsed_ = false;
            
            GPSSignalLevel gpsLevel_;
            
            int32_t satelliteCount_;
            
            int32_t remainingFlightTime_;
            
            int32_t batteryPercentageNeededToLandFromCurrentHeight_;
            
            int32_t batteryPercentageNeededToGoHome_;
            
            SmartRTHState smartRTHState_;
            
            ConnectionFailSafeBehavior behavior_;
            
            VirtualStickFlightControlDataSharedPtr virtualControlData_;
            
            bool isFailsafeEnabled_;
            
            bool areMotorsOn_;
            
            bool isHomeLocationSet_;
            
            bool isLandingConfirmationNeeded_;
            
            bool hasReachedMaxFlightHeight_;
            
            bool hasReachedMaxFlightRadius_;
            
            FlightWindWarning windWarning_;
            
            int32_t countOfFlights_;
            
            int32_t flightLogIndex_;
            
            bool isFlying_;
            
            double flightTimeInSeconds_ = 0;
            
            double cumulativeFlightDistance_ = 0;
            
            int32_t smartRTHCountdown_;
            
            uint8_t versionC_;
            
            uint8_t drone_type_;

            //Custom
            int32_t flyCCommand_;
            int32_t canIocWork_;
            int32_t isSwaveWork_;
            int32_t GoHomeStatus_;
            int32_t isVisionUsed_;
            int32_t batteryWarning_;
            int32_t isImuPreheated_;
            int32_t modeChannel_;
            int32_t isCompassError_;
            int32_t waveError_;
            int32_t batteryType_;
            int32_t isOutOfLimit_;
            int32_t isGoHomeHeightModified_;
            int32_t isPropellerCatapult_;
            int32_t isMotorBlocked_;
            int32_t isNotEnoughForce_;
            int32_t isBarometerDeadInAir_;
            int32_t isVibrating_;
            int32_t isAcceletorOverRange_;
            int32_t flighAction_;
            int32_t motorStartFailedCause_;
            int32_t nonGpsCause_;
            int32_t waypointLimitMode_;
            int32_t battery_;
            int32_t motorRevolution_;
            int32_t ImuInitFailReason_;
            int32_t ctrlDevice_;
        };
    }
}

#endif /* fr_standardization_flightcontroller_model_hpp */
