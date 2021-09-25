/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup ManualControl
 * @brief Interpretes the control input in ManualControlCommand
 * @{
 *
 * @file       stabilizedhandler.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2014.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************/
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

#include "inc/manualcontrol.h"
#include <mathmisc.h>
#include <sin_lookup.h>
#include <manualcontrolcommand.h>
#include <stabilizationdesired.h>
#include <flightmodesettings.h>
#include <stabilizationbank.h>
#include <flightstatus.h>
#include <optipositionstate.h>
#include <optivelocitystate.h>
#include <optisetpoint.h>
#include <optisetpointsettings.h>

// Private constants

// Private types

// Private functions
static float applyExpo(float value, float expo);
//static void handlePosHold(ManualControlCommandData cmd);
static void handleAltHoldTest(ManualControlCommandData cmd);
static void handleXYHold(ManualControlCommandData cmd);



// Private variables
static uint8_t currentFpvTiltAngle = 0;
static float cosAngle = 0.0f;
static float sinAngle = 0.0f;

/*
static bool initFlight = false; // alt/pos mode flight init

// position handler variable;
static int adjustPosXYTime = 0;
static bool isAdjustingPosXY = true;
static bool isAdjustingPosZ = false;
static float errorPosX = 0.f;
static float errorPosY = 0.f;
static float errorPosZ = 0.f;
*/

// threshold to avoid noise when poshold;
#define STICK_POSHOLD_THRES (0.03f)

static float applyExpo(float value, float expo)
{
    // note: fastPow makes a small error, therefore result needs to be bound
    float exp = boundf(fastPow(1.01395948f, expo), 0.25f, 4.0f);

    // magic number scales expo
    // so that
    // expo=100 yields value**10
    // expo=0 yields value**1
    // expo=-100 yields value**(1/10)
    // (pow(4.0,1/100)~=1.01395948)
    if (value > 0.0f) {
        return boundf(fastPow(value, exp), 0.0f, 1.0f);
    } else if (value < -0.0f) {
        return boundf(-fastPow(-value, exp), -1.0f, 0.0f);
    } else {
        return 0.0f;
    }
}


/**
 * @brief Handler to control Stabilized flightmodes. FlightControl is governed by "Stabilization"
 * @input: ManualControlCommand
 * @output: StabilizationDesired
 */
void stabilizedHandler(__attribute__((unused)) bool newinit)
{
    static bool inited = false;

    if (!inited) {
        inited = true;
        StabilizationDesiredInitialize();
        StabilizationBankInitialize();
    }

    ManualControlCommandData cmd;
    ManualControlCommandGet(&cmd);

    FlightModeSettingsData settings;
    FlightModeSettingsGet(&settings);

    StabilizationDesiredData stabilization;
    StabilizationDesiredGet(&stabilization);

    StabilizationBankData stabSettings;
    StabilizationBankGet(&stabSettings);

    cmd.Roll  = applyExpo(cmd.Roll, stabSettings.StickExpo.Roll);
    cmd.Pitch = applyExpo(cmd.Pitch, stabSettings.StickExpo.Pitch);
    cmd.Yaw   = applyExpo(cmd.Yaw, stabSettings.StickExpo.Yaw);

    if (stabSettings.FpvCamTiltCompensation > 0) {
        // Reduce Cpu load
        if (currentFpvTiltAngle != stabSettings.FpvCamTiltCompensation) {
            cosAngle = cos_lookup_deg((float)stabSettings.FpvCamTiltCompensation);
            sinAngle = sin_lookup_deg((float)stabSettings.FpvCamTiltCompensation);
            currentFpvTiltAngle = stabSettings.FpvCamTiltCompensation;
        }
        float rollCommand = cmd.Roll;
        float yawCommand  = cmd.Yaw;

        // http://shrediquette.blogspot.de/2016/01/some-thoughts-on-camera-tilt.html
        // When Roll right, add negative Yaw.
        // When Yaw left, add negative Roll.
        cmd.Roll = boundf((cosAngle * rollCommand) + (sinAngle * yawCommand), -1.0f, 1.0f);
        cmd.Yaw  = boundf((cosAngle * yawCommand) - (sinAngle * rollCommand), -1.0f, 1.0f);
    }

    uint8_t *stab_settings;
    FlightStatusData flightStatus;
    FlightStatusGet(&flightStatus);
    switch (flightStatus.FlightMode) {
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
        stab_settings = (uint8_t *)FlightModeSettingsStabilization1SettingsToArray(settings.Stabilization1Settings);
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
        stab_settings = (uint8_t *)FlightModeSettingsStabilization2SettingsToArray(settings.Stabilization2Settings);
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
        stab_settings = (uint8_t *)FlightModeSettingsStabilization3SettingsToArray(settings.Stabilization3Settings);
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED4:
        stab_settings = (uint8_t *)FlightModeSettingsStabilization4SettingsToArray(settings.Stabilization4Settings);
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED5:
        stab_settings = (uint8_t *)FlightModeSettingsStabilization5SettingsToArray(settings.Stabilization5Settings);
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED6:
        stab_settings = (uint8_t *)FlightModeSettingsStabilization6SettingsToArray(settings.Stabilization6Settings);
        break;
#if !defined(PIOS_EXCLUDE_ADVANCED_FEATURES)
    case FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE:
        // let autotune.c handle it
        // because it must switch to Attitude after <user configurable> seconds
        return;

#endif /* !defined(PIOS_EXCLUDE_ADVANCED_FEATURES) */
    default:
        // Major error, this should not occur because only enter this block when one of these is true
        AlarmsSet(SYSTEMALARMS_ALARM_MANUALCONTROL, SYSTEMALARMS_ALARM_CRITICAL);
        stab_settings = (uint8_t *)FlightModeSettingsStabilization1SettingsToArray(settings.Stabilization1Settings);
        return;
    }
	// stablelize des roll angle
	// handle flight mode,stablelized2 is poshold --- dxx 20210617-11.01
	switch (flightStatus.FlightMode) {
		case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
			//handlePosHold(cmd);
			handleAltHoldTest(cmd);
			handleXYHold(cmd);
			//if stab2 continue set stabilization desired to maintain stable;
			//return;
		default:
    		stabilization.Roll = (stab_settings[0] == STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE) ? cmd.Roll * stabSettings.RollMax :
    					0;
			stabilization.Pitch = (stab_settings[1] == STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE) ? cmd.Pitch * stabSettings.PitchMax :
						0;
			// TOOD: Add assumption about order of stabilization desired and manual control stabilization mode fields having same order
    		stabilization.StabilizationMode.Roll  = stab_settings[0];
    		stabilization.StabilizationMode.Pitch = stab_settings[1];
			stabilization.Yaw = (stab_settings[2] == STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE) ? cmd.Yaw * stabSettings.YawMax :
								(stab_settings[2] == STABILIZATIONDESIRED_STABILIZATIONMODE_RATE) ? cmd.Yaw * stabSettings.ManualRate.Yaw :
								0;
			stabilization.StabilizationMode.Yaw = stab_settings[2];
			stabilization.StabilizationMode.Thrust = stab_settings[3];
    		stabilization.Thrust = cmd.Thrust;
    		StabilizationDesiredSet(&stabilization);
			break;
		}
}

static void handleAltHoldTest(ManualControlCommandData cmd)
{
	OptiSetpointData OptiSetpoint;
	OptiSetpointSettingsData OptiSetpointSettings;
	OptiPositionStateData OptiPositionState;
	OptiVelocityStateData OptiVelocityState;
	OptiSetpointSettingsGet(&OptiSetpointSettings);

	OptiPositionStateGet(&OptiPositionState);
	OptiVelocityStateGet(&OptiVelocityState);
	OptiSetpointGet(&OptiSetpoint);

	//////////////////////////z control///////////////////////////////////
	// set in const 0.8m 
	if(cmd.Thrust>=.5f)
	{
		OptiSetpoint.OptiSetpointMode.Down = OPTISETPOINT_OPTISETPOINTMODE_POSITION;
		OptiSetpoint.Position.Down = -0.8f;	/*调整新位置*/	
	}
	else
	{
		OptiSetpoint.OptiSetpointMode.Down = OPTISETPOINT_OPTISETPOINTMODE_DISABLE;
	}
	OptiSetpointSet(&OptiSetpoint);
}

static void handleXYHold(ManualControlCommandData cmd)
{
    
	OptiSetpointData OptiSetpoint;
	OptiSetpointSettingsData OptiSetpointSettings;
	OptiPositionStateData OptiPositionState;
	OptiVelocityStateData OptiVelocityState;
	OptiSetpointSettingsGet(&OptiSetpointSettings);

	OptiPositionStateGet(&OptiPositionState);
	OptiVelocityStateGet(&OptiVelocityState);
	OptiSetpointGet(&OptiSetpoint);

	static bool isAdjustingPosXY = true;
	static uint8_t adjustPosXYTime = 0;

	///////////////////////////////////////////xy control///////////////////////
	//int send_vel = OptiSetpointSettings.OptiSpeedMax.North*10;
	//DEBUG_PRINTF(2,"POSE VEL %d\n",(int)(maxPosVelLimit.MaxVelocityNorth*10));
	//DEBUG_PRINTF(2,"POSE VEL2 %d\n",send_vel);
	// TODO: add if optidata vailable;
	if(fabsf(cmd.Roll)>STICK_POSHOLD_THRES||fabsf(cmd.Pitch)>STICK_POSHOLD_THRES)
	{
		float originalVy = cmd.Roll * maxPosVelLimit.MaxVelocityNorth;
		float originalVx = cmd.Pitch * maxPosVelLimit.MaxVelocityEast;
		
		adjustPosXYTime = 0;
		isAdjustingPosXY = true;
		
		OptiSetpoint.OptiSetpointMode.North = OPTISETPOINT_OPTISETPOINTMODE_VELOCITY;
		OptiSetpoint.OptiSetpointMode.East = OPTISETPOINT_OPTISETPOINTMODE_VELOCITY;
		// opti coord 2 yaw coord
		OptiSetpoint.Velocity.North = originalVx;
		OptiSetpoint.Velocity.East = originalVy;
	}
	else if(isAdjustingPosXY == true)
	{
		if(adjustPosXYTime++ > 100)
		{
			adjustPosXYTime = 0;
			isAdjustingPosXY = false;
		}		
		OptiSetpoint.OptiSetpointMode.North = OPTISETPOINT_OPTISETPOINTMODE_POSITION;
		OptiSetpoint.OptiSetpointMode.East = OPTISETPOINT_OPTISETPOINTMODE_POSITION;
		OptiSetpoint.Position.North = 0;//OptiPositionState.North;	//调整新位置
		OptiSetpoint.Position.East = 0;//OptiPositionState.East;	//调整新位置
	}
	OptiSetpointSet(&OptiSetpoint);
}

// positionhold mannual handler, set Positiondesired;
// cmd 
/*
static void handlePosHold(ManualControlCommandData cmd)
{
	OptiSetpointData OptiSetpoint;
	OptiSetpointSettingsData OptiSetpointSettings;
	OptiPositionStateData OptiPositionState;
	OptiVelocityStateData OptiVelocityState;
	OptiSetpointSettingsGet(&OptiSetpointSettings);

	OptiPositionStateGet(&OptiPositionState);
	OptiVelocityStateGet(&OptiVelocityState);
	OptiSetpointGet(&OptiSetpoint);

	//////////////////////////z control///////////////////////////////////
	float climb = ((cmd.Thrust - .5f) / .5f); //中立位置为0.5
	float check_deadband = climb;
	const float DEADBAND      = 0.20f;
    const float DEADBAND_HIGH = DEADBAND;
    const float DEADBAND_LOW  =  -DEADBAND ;

	//DEBUG_PRINTF(2,"Pos hold: High: %d,low: %d\n",(int)(DEADBAND_HIGH*10),(int)(DEADBAND_LOW*10));

	if(climb > 0.f) 
	{
		climb *= maxPosVelLimit.MaxVelocityUp;
		if(!initFlight)
		{
			initFlight = true;
			isAdjustingPosXY = true;
			errorPosX = 0.f;
			errorPosY = 0.f;
			errorPosZ = 0.f;
			DEBUG_PRINTF(2,"POS HOLD ENABLE\n");
		}
	}
	else
		climb *= maxPosVelLimit.MaxVelocityDown;
	if(initFlight)
	{
		if (check_deadband<DEADBAND_LOW||check_deadband>DEADBAND_HIGH)
		{
			isAdjustingPosZ = true;												
			OptiSetpoint.OptiSetpointMode.Down = OPTISETPOINT_OPTISETPOINTMODE_VELOCITY;
			OptiSetpoint.Velocity.Down = -climb; //down is positive
		}
		else if (isAdjustingPosZ == true)
		{
			isAdjustingPosZ = false;
			OptiSetpoint.OptiSetpointMode.Down = OPTISETPOINT_OPTISETPOINTMODE_POSITION;
			OptiSetpoint.Position.Down = OptiPositionState.Down + errorPosZ;	//调整新位置									
		}
		else if(isAdjustingPosZ == false)	//Z位移误差
		{
		// TODO: test if maxPosVelLimit.MaxPositionDown is available--dxx
			OptiSetpoint.OptiSetpointMode.Down = OPTISETPOINT_OPTISETPOINTMODE_POSITION;
			errorPosZ = OptiSetpoint.Position.Down - OptiPositionState.Down;
			errorPosZ = boundf(errorPosZ, -maxPosVelLimit.MaxPositionDown, maxPosVelLimit.MaxPositionDown);	//误差限幅 单位m
		}
	}
	else//着陆状态
	{
		OptiSetpoint.OptiSetpointMode.Down = OPTISETPOINT_OPTISETPOINTMODE_DISABLE;
		OptiSetpoint.Trust = 0.f;
		OptiSetpoint.Velocity.Down = 0.f;
		OptiSetpoint.Position.Down = 0.f;
		initFlight = false;
		isAdjustingPosZ = false;
	}

	///////////////////////////////////////////xy control///////////////////////
	//int send_vel = OptiSetpointSettings.OptiSpeedMax.North*10;
	//DEBUG_PRINTF(2,"POSE VEL %d\n",(int)(maxPosVelLimit.MaxVelocityNorth*10));
	//DEBUG_PRINTF(2,"POSE VEL2 %d\n",send_vel);
	// TODO: add if optidata vailable;
	if(fabsf(cmd.Roll)>STICK_POSHOLD_THRES||fabsf(cmd.Pitch)>STICK_POSHOLD_THRES)
	{
		float originalVy = cmd.Roll * maxPosVelLimit.MaxVelocityNorth;
		float originalVx = cmd.Pitch * maxPosVelLimit.MaxVelocityEast;

		float cosyaw = cosf(OptiPositionState.Yaw);
		float sinyaw = sinf(OptiPositionState.Yaw);
		
		adjustPosXYTime = 0;
		isAdjustingPosXY = true;
		
		OptiSetpoint.OptiSetpointMode.North = OPTISETPOINT_OPTISETPOINTMODE_VELOCITY;
		OptiSetpoint.OptiSetpointMode.East = OPTISETPOINT_OPTISETPOINTMODE_VELOCITY;
		// opti coord 2 yaw coord
		OptiSetpoint.Velocity.North = originalVx * cosyaw + originalVy * sinyaw;
		OptiSetpoint.Velocity.East = originalVy * cosyaw - originalVx * sinyaw;

	}
	else if(isAdjustingPosXY == true)
	{
		if(adjustPosXYTime++ > 100)
		{
			adjustPosXYTime = 0;
			isAdjustingPosXY = false;
		}		
		OptiSetpoint.OptiSetpointMode.North = OPTISETPOINT_OPTISETPOINTMODE_POSITION;
		OptiSetpoint.OptiSetpointMode.East = OPTISETPOINT_OPTISETPOINTMODE_POSITION;
		OptiSetpoint.Position.North = OptiPositionState.North + errorPosX;	//调整新位置
		OptiSetpoint.Position.East = OptiPositionState.East + errorPosY;	//调整新位置
	}
	else 
	{
		errorPosX = OptiSetpoint.Position.North - OptiPositionState.North;
		errorPosY = OptiSetpoint.Position.East - OptiPositionState.East;
		errorPosX = boundf(errorPosX, -maxPosVelLimit.MaxPositionNorth, maxPosVelLimit.MaxPositionNorth);	//误差限幅 单位m
		errorPosY = boundf(errorPosY, -maxPosVelLimit.MaxPositionEast, maxPosVelLimit.MaxPositionEast);	//误差限幅 单位m
	}
	OptiSetpointSet(&OptiSetpoint);
}
*/
/**
 * @}
 * @}
 */
