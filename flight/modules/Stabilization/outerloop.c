/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup StabilizationModule Stabilization Module
 * @brief Stabilization PID loops in an airframe type independent manner
 * @note This object updates the @ref ActuatorDesired "Actuator Desired" based on the
 * PID loops on the @ref AttitudeDesired "Attitude Desired" and @ref AttitudeState "Attitude State"
 * @{
 *
 * @file       outerloop.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2015.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2014.
 * @brief      Attitude stabilization module.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <openpilot.h>
#include <pid.h>
#include <callbackinfo.h>
#include <ratedesired.h>
#include <stabilizationdesired.h>
#include <attitudestate.h>
#include <stabilizationstatus.h>
#include <flightstatus.h>
#include <manualcontrolcommand.h>
#include <stabilizationbank.h>


#include <stabilization.h>
#include <cruisecontrol.h>
#include <altitudeloop.h>
#include <CoordinateConversions.h>

#include <optipositionstate.h>
#include <optivelocitystate.h>
#include <optisetpoint.h>
#include <optisetpointsettings.h>


// Private constants

#define CALLBACK_PRIORITY CALLBACK_PRIORITY_REGULAR

#define UPDATE_EXPECTED   (1.0f / PIOS_SENSOR_RATE)
#define UPDATE_MIN        1.0e-6f
#define UPDATE_MAX        1.0f
#define UPDATE_ALPHA      1.0e-2f

#define CONSTANT_ONE_GRAVITY 9.81f
#define ACC_XY_MAX 10.f

#define POS_XY_CONTROL

// Private variables
static DelayedCallbackInfo *callbackHandle;
static AttitudeStateData attitude;

static uint8_t previous_mode[AXES] = { 255, 255, 255, 255 };
static PiOSDeltatimeConfig timeval;
static bool pitchMin = false;
static bool pitchMax = false;
static bool rollMin  = false;
static bool rollMax  = false;

// Private functions
static void stabilizationOuterloopTask();
static void AttitudeStateUpdatedCb(__attribute__((unused)) UAVObjEvent *ev);
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES

#ifdef POS_XY_CONTROL
static void PositionHoldXY( float dT);
static void DisablePositionHoldXY();
#endif

#endif

void stabilizationOuterloopInit()
{
    RateDesiredInitialize();
    StabilizationDesiredInitialize();
    AttitudeStateInitialize();
    StabilizationStatusInitialize();
    FlightStatusInitialize();
    ManualControlCommandInitialize();

	OptiVelocityStateInitialize();
	OptiPositionStateInitialize();
	OptiSetpointInitialize();
	OptiSetpointSettingsInitialize();

	//OptiSetpointConnectCallback(&SettingsUpdatedCb);

    PIOS_DELTATIME_Init(&timeval, UPDATE_EXPECTED, UPDATE_MIN, UPDATE_MAX, UPDATE_ALPHA);

    callbackHandle = PIOS_CALLBACKSCHEDULER_Create(&stabilizationOuterloopTask, CALLBACK_PRIORITY, CBTASK_PRIORITY, CALLBACKINFO_RUNNING_STABILIZATION0, STACK_SIZE_BYTES);
    AttitudeStateConnectCallback(AttitudeStateUpdatedCb);
}


/**
 * WARNING! This callback executes with critical flight control priority every
 * time a gyroscope update happens do NOT put any time consuming calculations
 * in this loop unless they really have to execute with every gyro update
 */
static void stabilizationOuterloopTask()
{
	
    AttitudeStateData attitudeState;
    RateDesiredData rateDesired;
    StabilizationDesiredData stabilizationDesired;
    StabilizationStatusOuterLoopData enabled;
	FlightStatusData flightStatus;

	OptiPositionStateData optiPositionState;
	OptiPositionStateGet(&optiPositionState);


    AttitudeStateGet(&attitudeState);
    StabilizationDesiredGet(&stabilizationDesired);
    RateDesiredGet(&rateDesired);
    StabilizationStatusOuterLoopGet(&enabled);
	FlightStatusGet(&flightStatus);
    float *stabilizationDesiredAxis = &stabilizationDesired.Roll;
    float *rateDesiredAxis = &rateDesired.Roll;
    int t;
    float dT    = PIOS_DELTATIME_GetAverageSeconds(&timeval);
    StabilizationStatusOuterLoopOptions newThrustMode = StabilizationStatusOuterLoopToArray(enabled)[STABILIZATIONSTATUS_OUTERLOOP_THRUST];
    bool reinit = (newThrustMode != previous_mode[STABILIZATIONSTATUS_OUTERLOOP_THRUST]);
	
	
	
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
	uint8_t new_mode = flightStatus.FlightMode;
	
	// TODO: adjust position
	if(new_mode == FLIGHTSTATUS_FLIGHTMODE_STABILIZED2)
	{
		float desired_roll,desired_pitch;
		rateDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST] = stabilizationAltitudeHold(stabilizationDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST], ALTITUDEHOLD, reinit);
		// pos xy ctrl
		PositionHoldXY(dT);
		StabilizationDesiredRollGet(&desired_roll);
		StabilizationDesiredPitchGet(&desired_pitch);
		stabilizationDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_ROLL] = desired_roll;
		stabilizationDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_PITCH] = desired_pitch;
		if(fabsf(optiPositionState.North)>2.0f||fabsf(optiPositionState.East)>2.0f||optiPositionState.Down<=-2.0f)
		{
		    rateDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST] = 0;//stabilizationAltitudeHold(stabilizationDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST], ALTITUDEHOLD, reinit);
		}

		// end pos ctrl
		
	}else
	{
		stabilizationDisableAltitudeHold();
	#ifdef POS_XY_CONTROL
		DisablePositionHoldXY();
	#endif
		rateDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST] = stabilizationDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST];
		
	}
	#else
		
		rateDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST] = stabilizationDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST];
	#endif	



/*
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    // Trigger a disable message to the alt hold on reinit to prevent that loop from running when not in use.
    if (reinit) {
        if (previous_mode[STABILIZATIONSTATUS_OUTERLOOP_THRUST] == STABILIZATIONSTATUS_OUTERLOOP_ALTITUDE ||
            previous_mode[STABILIZATIONSTATUS_OUTERLOOP_THRUST] == STABILIZATIONSTATUS_OUTERLOOP_ALTITUDEVARIO) {
            if (newThrustMode != STABILIZATIONSTATUS_OUTERLOOP_ALTITUDE && newThrustMode != STABILIZATIONSTATUS_OUTERLOOP_ALTITUDEVARIO) {
                // disable the altvario velocity control loop
                stabilizationDisableAltitudeHold();
            }
        }
    }
#endif
    // update previous mode
    previous_mode[STABILIZATIONSTATUS_OUTERLOOP_THRUST] = newThrustMode;

    // calculate the thrust desired
    switch (newThrustMode) {
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    case STABILIZATIONSTATUS_OUTERLOOP_ALTITUDE:
        rateDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST] = stabilizationAltitudeHold(stabilizationDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST], ALTITUDEHOLD, reinit);
        break;
    case STABILIZATIONSTATUS_OUTERLOOP_ALTITUDEVARIO:
        rateDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST] = stabilizationAltitudeHold(stabilizationDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST], ALTITUDEVARIO, reinit);
        break;
#endif
    case STABILIZATIONSTATUS_OUTERLOOP_DIRECT:
    case STABILIZATIONSTATUS_OUTERLOOP_DIRECTWITHLIMITS:
    default:
        rateDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST] = stabilizationDesiredAxis[STABILIZATIONSTATUS_OUTERLOOP_THRUST];
        break;
    }
*/
	
	
    float local_error[3];
    {
#if defined(PIOS_QUATERNION_STABILIZATION)
        // Quaternion calculatifaon of error in each axis.  Uses more memory.
        float rpy_desired[3];
        float q_desired[4];
        float q_error[4];

        for (t = 0; t < 3; t++) {
            switch (StabilizationStatusOuterLoopToArray(enabled)[t]) {
            case STABILIZATIONSTATUS_OUTERLOOP_ATTITUDE:
            case STABILIZATIONSTATUS_OUTERLOOP_RATTITUDE:
            case STABILIZATIONSTATUS_OUTERLOOP_WEAKLEVELING:
                rpy_desired[t] = stabilizationDesiredAxis[t];
                break;
            case STABILIZATIONSTATUS_OUTERLOOP_DIRECTWITHLIMITS:
            case STABILIZATIONSTATUS_OUTERLOOP_DIRECT:
            default:
                rpy_desired[t] = ((float *)&attitudeState.Roll)[t];
                break;
            }
        }

        RPY2Quaternion(rpy_desired, q_desired);
        quat_inverse(q_desired);
        quat_mult(q_desired, &attitudeState.q1, q_error);
        quat_inverse(q_error);
        Quaternion2RPY(q_error, local_error);

#else /* if defined(PIOS_QUATERNION_STABILIZATION) */
        // Simpler algorithm for CC, less memory
        local_error[0] = stabilizationDesiredAxis[0] - attitudeState.Roll;
        local_error[1] = stabilizationDesiredAxis[1] - attitudeState.Pitch;
        local_error[2] = stabilizationDesiredAxis[2] - attitudeState.Yaw;

        // find shortest way
        float modulo = fmodf(local_error[2] + 180.0f, 360.0f);
        if (modulo < 0) {
            local_error[2] = modulo + 180.0f;
        } else {
            local_error[2] = modulo - 180.0f;
        }
#endif /* if defined(PIOS_QUATERNION_STABILIZATION) */
    }

	
    for (t = STABILIZATIONSTATUS_OUTERLOOP_ROLL; t < STABILIZATIONSTATUS_OUTERLOOP_THRUST; t++) {
        reinit = (StabilizationStatusOuterLoopToArray(enabled)[t] != previous_mode[t]);
        previous_mode[t] = StabilizationStatusOuterLoopToArray(enabled)[t];

        if (reinit) {
            stabSettings.outerPids[t].iAccumulator = 0;
        }
        switch (StabilizationStatusOuterLoopToArray(enabled)[t]) {
        case STABILIZATIONSTATUS_OUTERLOOP_ATTITUDE:
            rateDesiredAxis[t] = pid_apply(&stabSettings.outerPids[t], local_error[t], dT);
            break;
        case STABILIZATIONSTATUS_OUTERLOOP_RATTITUDE:
        {
            float stickinput[3];
            stickinput[0] = boundf(stabilizationDesiredAxis[0] / stabSettings.stabBank.RollMax, -1.0f, 1.0f);
            stickinput[1] = boundf(stabilizationDesiredAxis[1] / stabSettings.stabBank.PitchMax, -1.0f, 1.0f);
            stickinput[2] = boundf(stabilizationDesiredAxis[2] / stabSettings.stabBank.YawMax, -1.0f, 1.0f);
            float rateDesiredAxisRate = stickinput[t] * StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[t];
            // limit corrective rate to maximum rates to not give it overly large impact over manual rate when joined together
            rateDesiredAxis[t] = boundf(pid_apply(&stabSettings.outerPids[t], local_error[t], dT),
                                        -StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[t],
                                        StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[t]
                                        );
            // Compute the weighted average rate desired
            // Using max() rather than sqrt() for cpu speed;
            // - this makes the stick region into a square;
            // - this is a feature!
            // - hold a roll angle and add just pitch without the stick sensitivity changing
            float magnitude = fabsf(stickinput[t]);
            if (t < 2) {
                magnitude = fmaxf(fabsf(stickinput[0]), fabsf(stickinput[1]));
            }

            // modify magnitude to move the Att to Rate transition to the place
            // specified by the user
            // we are looking for where the stick angle == transition angle
            // and the Att rate equals the Rate rate
            // that's where Rate x (1-StickAngle) [Attitude pulling down max X Ratt proportion]
            // == Rate x StickAngle [Rate pulling up according to stick angle]
            // * StickAngle [X Ratt proportion]
            // so 1-x == x*x or x*x+x-1=0 where xE(0,1)
            // (-1+-sqrt(1+4))/2 = (-1+sqrt(5))/2
            // and quadratic formula says that is 0.618033989f
            // I tested 14.01 and came up with .61 without even remembering this number
            // I thought that moving the P,I, and maxangle terms around would change this value
            // and that I would have to take these into account, but varying
            // all P's and I's by factors of 1/2 to 2 didn't change it noticeably
            // and varying maxangle from 4 to 120 didn't either.
            // so for now I'm not taking these into account
            // while working with this, it occurred to me that Attitude mode,
            // set up with maxangle=190 would be similar to Ratt, and it is.
#define STICK_VALUE_AT_MODE_TRANSITION 0.618033989f

            // the following assumes the transition would otherwise be at 0.618033989f
            // and THAT assumes that Att ramps up to max roll rate
            // when a small number of degrees off of where it should be

            // if below the transition angle (still in attitude mode)
            // '<=' instead of '<' keeps rattitude_mode_transition_stick_position==1.0 from causing DZ
            if (magnitude <= stabSettings.rattitude_mode_transition_stick_position) {
                magnitude *= STICK_VALUE_AT_MODE_TRANSITION / stabSettings.rattitude_mode_transition_stick_position;
            } else {
                magnitude = (magnitude - stabSettings.rattitude_mode_transition_stick_position)
                            * (1.0f - STICK_VALUE_AT_MODE_TRANSITION)
                            / (1.0f - stabSettings.rattitude_mode_transition_stick_position)
                            + STICK_VALUE_AT_MODE_TRANSITION;
            }
            rateDesiredAxis[t] = (1.0f - magnitude) * rateDesiredAxis[t] + magnitude * rateDesiredAxisRate;
        }
        break;
        case STABILIZATIONSTATUS_OUTERLOOP_WEAKLEVELING:
            // FIXME: local_error[] is rate - attitude for Weak Leveling
            // The only ramifications are:
            // Weak Leveling Kp is off by a factor of 3 to 12 and may need a different default in GCS
            // Changing Rate mode max rate currently requires a change to Kp
            // That would be changed to Attitude mode max angle affecting Kp
            // Also does not take dT into account
        {
            float stickinput[3];
            stickinput[0] = boundf(stabilizationDesiredAxis[0] / stabSettings.stabBank.RollMax, -1.0f, 1.0f);
            stickinput[1] = boundf(stabilizationDesiredAxis[1] / stabSettings.stabBank.PitchMax, -1.0f, 1.0f);
            stickinput[2] = boundf(stabilizationDesiredAxis[2] / stabSettings.stabBank.YawMax, -1.0f, 1.0f);
            float rate_input    = stickinput[t] * StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[t];
            float weak_leveling = local_error[t] * stabSettings.settings.WeakLevelingKp;
            weak_leveling = boundf(weak_leveling, -stabSettings.settings.MaxWeakLevelingRate, stabSettings.settings.MaxWeakLevelingRate);

            // Compute desired rate as input biased towards leveling
            rateDesiredAxis[t] = rate_input + weak_leveling;
        }
        break;
        case STABILIZATIONSTATUS_OUTERLOOP_DIRECTWITHLIMITS:
            rateDesiredAxis[t] = stabilizationDesiredAxis[t]; // default for all axes
            // now test limits for pitch and/or roll
            if (t == 1) { // pitch
                if ((attitudeState.Pitch < -stabSettings.stabBank.PitchMax) || pitchMin) {
                    pitchMin = true;
                    // Attitude exceeds pitch min,
                    // Do Attitude stabilisation at min pitch angle while user still maintain negative pitch
                    if (stabilizationDesiredAxis[t] < 0.0f) {
                        local_error[t]     = -stabSettings.stabBank.PitchMax - attitudeState.Pitch;
                        rateDesiredAxis[t] = pid_apply(&stabSettings.outerPids[t], local_error[t], dT);
                    } else {
                        // Stop Attitude stabilization and return to Rate
                        pitchMin = false;
                    }
                } else if ((attitudeState.Pitch > stabSettings.stabBank.PitchMax) || pitchMax) {
                    pitchMax = true;
                    // Attitude exceeds pitch max
                    // Do Attitude stabilisation at max pitch angle while user still maintain positive pitch
                    if (stabilizationDesiredAxis[t] > 0.0f) {
                        local_error[t]     = stabSettings.stabBank.PitchMax - attitudeState.Pitch;
                        rateDesiredAxis[t] = pid_apply(&stabSettings.outerPids[t], local_error[t], dT);
                    } else {
                        // Stop Attitude stabilization and return to Rate
                        pitchMax = false;
                    }
                }
            } else if (t == 0) { // roll
                if ((attitudeState.Roll < -stabSettings.stabBank.RollMax) || rollMin) {
                    rollMin = true;
                    // Attitude exceeds roll min,
                    // Do Attitude stabilisation at min roll angle while user still maintain negative roll
                    if (stabilizationDesiredAxis[t] < 0.0f) {
                        local_error[t]     = -stabSettings.stabBank.RollMax - attitudeState.Roll;
                        rateDesiredAxis[t] = pid_apply(&stabSettings.outerPids[t], local_error[t], dT);
                    } else {
                        // Stop Attitude stabilization and return to Rate
                        rollMin = false;
                    }
                } else if ((attitudeState.Roll > stabSettings.stabBank.RollMax) || rollMax) {
                    rollMax = true;
                    // Attitude exceeds roll max
                    // Do Attitude stabilisation at max roll angle while user still maintain positive roll
                    if (stabilizationDesiredAxis[t] > 0.0f) {
                        local_error[t]     = stabSettings.stabBank.RollMax - attitudeState.Roll;
                        rateDesiredAxis[t] = pid_apply(&stabSettings.outerPids[t], local_error[t], dT);
                    } else {
                        // Stop Attitude stabilization and return to Rate
                        rollMax = false;
                    }
                }
            }
            break;

        case STABILIZATIONSTATUS_OUTERLOOP_DIRECT:
        default:
            rateDesiredAxis[t] = stabilizationDesiredAxis[t];
            break;
        }
    }

    RateDesiredSet(&rateDesired);
    {
        FlightStatusArmedOptions armed;
        FlightStatusArmedGet(&armed);
        float throttleDesired;
        ManualControlCommandThrottleGet(&throttleDesired);
        if (armed != FLIGHTSTATUS_ARMED_ARMED ||
            ((stabSettings.settings.LowThrottleZeroIntegral == STABILIZATIONSETTINGS_LOWTHROTTLEZEROINTEGRAL_TRUE) && throttleDesired < 0)) {
            // Force all axes to reinitialize when engaged
            for (t = 0; t < AXES; t++) {
                previous_mode[t] = 255;
            }
        }
    }

// update cruisecontrol based on attitude
    cruisecontrol_compute_factor(&attitudeState, rateDesired.Thrust);
    stabSettings.monitor.rateupdates = 0;
}
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES

#ifdef POS_XY_CONTROL
static void AccelControl(float desired_acceln,float desired_accele)
{
	//CONSTANT_ONE_GRAVITY
	AttitudeStateData attitudeState;
	AttitudeStateGet(&attitudeState);

	float cos_yaw = cosf(DEG2RAD(attitudeState.Yaw));
	float sin_yaw = sinf(DEG2RAD(attitudeState.Yaw));
	
	float desired_accel_forward= desired_acceln * cos_yaw + desired_accele*sin_yaw;
	float desired_accel_right = -desired_acceln * sin_yaw + desired_accele*cos_yaw;

	// assume down accel is one g up = -9.81, calcuate lean angle
	float desired_pitch_angle_rad = -atanf(desired_accel_forward/CONSTANT_ONE_GRAVITY);
	float desired_roll_angle_rad = atanf(desired_accel_right*cosf(-desired_pitch_angle_rad)/CONSTANT_ONE_GRAVITY); 

	float desired_roll = RAD2DEG(desired_roll_angle_rad);
	float desired_pitch = RAD2DEG(desired_pitch_angle_rad);
	desired_roll = boundf(desired_roll, -stabSettings.stabBank.RollMax, stabSettings.stabBank.RollMax);
    desired_pitch = boundf(desired_pitch, -stabSettings.stabBank.PitchMax, stabSettings.stabBank.PitchMax);
	
    StabilizationDesiredRollSet(&desired_roll);
	StabilizationDesiredPitchSet(&desired_pitch);
}


static void VelocityControl(float desired_veln,float desired_vele,float dT)
{
	OptiVelocityStateData optiVelocityState;
	OptiVelocityStateGet(&optiVelocityState);

	float vel_errn = desired_veln - optiVelocityState.North;
	float vel_erre = desired_vele - optiVelocityState.East;
	
	float desired_acceln = pid_apply(&stabSettings.velocityPids[0],vel_errn,dT);
	desired_acceln = boundf(desired_acceln, -ACC_XY_MAX, ACC_XY_MAX);
	float desired_accele = pid_apply(&stabSettings.velocityPids[1],vel_erre,dT);
	desired_accele = boundf(desired_accele, -ACC_XY_MAX, ACC_XY_MAX);
	AccelControl(desired_acceln,desired_accele);
}

static void PositionHoldXY(float dT)
{
	OptiPositionStateData optiPositionState;
	OptiSetpointData optiSetpoint;

	float desired_veln = 0.f;
	float desired_vele = 0.f;
	
	OptiPositionStateGet(&optiPositionState);
	OptiSetpointGet(&optiSetpoint);

	//float cosyaw = cosf(optiPositionState.Yaw);
	//float sinyaw = sinf(optiPositionState.Yaw);
	
	desired_veln = optiSetpoint.Velocity.North;
	desired_vele = optiSetpoint.Velocity.East;

	
	if(optiPositionState.OptiDataVaild == OPTIPOSITIONSTATE_OPTIDATAVAILD_FALSE)
	{
		return;
	}
	
	if(optiSetpoint.OptiSetpointMode.North == OPTISETPOINT_OPTISETPOINTMODE_POSITION
		&&optiSetpoint.OptiSetpointMode.East == OPTISETPOINT_OPTISETPOINTMODE_POSITION)
	{
		float errn = optiSetpoint.Position.North - optiPositionState.North;
		float erre = optiSetpoint.Position.East - optiPositionState.East;
		
		float originalVx = pid_apply(&stabSettings.positionPids[0],errn,dT);
		float originalVy = pid_apply(&stabSettings.positionPids[1],erre,dT);

		// opti coord 2 yaw coord
		desired_veln = originalVx;
		desired_vele = originalVy;
	}
	VelocityControl(desired_veln,desired_vele,dT);
	
}

static void DisablePositionHoldXY()
{
  pid_zero(&stabSettings.positionPids[0]);
  pid_zero(&stabSettings.positionPids[1]);

  pid_zero(&stabSettings.velocityPids[0]);
  pid_zero(&stabSettings.velocityPids[1]);
}
#endif

#endif


static void AttitudeStateUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
#ifndef STABILIZATION_ATTITUDE_DOWNSAMPLED
    // to reduce CPU utilization, outer loop is not executed on every state update
    static uint8_t cpusaver = 0;

    if ((cpusaver++ % OUTERLOOP_SKIPCOUNT) == 0) {
#endif
    // this does not need mutex protection as both eventdispatcher and stabi run in same callback task!
    AttitudeStateGet(&attitude);
    PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);

#ifndef STABILIZATION_ATTITUDE_DOWNSAMPLED
}
#endif
}

/**
 * @}
 * @}
 */
