/* Avoid uncontrolled motor start */
while (false == throttleStick_startedDown) {

    /* Read throttle input from radio controller */
    reference_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

    if (15 > reference_throttle) {

        throttleStick_startedDown = true;

    } else {

        throttleStick_startedDown = false;
    }
}

/* Check if ESCs are enabled (Switch B on radio controller) */
if (500 <= FSA8S_ReadChannel(rc_controller, CHANNEL_6)) {
    ESC_isEnabled = true;
} else {
    ESC_isEnabled = false;
}

/* Turn off motors in case ESCs are disabled */
if (false == ESC_isEnabled) {

    /* Save motors speed */
    ESC_speeds[1] = 0;
    ESC_speeds[2] = 0;
    ESC_speeds[3] = 0;
    ESC_speeds[4] = 0;

    /* Turn off motors */
    ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
    ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
    ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
    ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);

    /* Reset PID variables */
    CSM_ResetPID();

} else {

    /* Check if timer has expired */
    if (Timer_Flag_ControlSystem) {

        /* Read input throttle from radio controller */
        reference_throttle = FSA8S_ReadChannel(rc_controller, CHANNEL_3);

        /* Check if throttle stick is low */
        if (CONTROL_SYSTEM_MINIMUM_INPUT_THROTTLE > reference_throttle) {

            /* Save motors speed */
            ESC_speeds[1] = 0;
            ESC_speeds[2] = 0;
            ESC_speeds[3] = 0;
            ESC_speeds[4] = 0;

            /* Turn off motors */
            ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
            ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
            ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
            ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);

            /* Reset PID variables */
            CSM_ResetPID();

        } else {

            /* Read GY-87 gyroscope sensor */
            GY87_ReadGyroscope(hgy87, &GY87_gyroscopeValues);
            /* Read GY-87 accelerometer sensor */
            GY87_ReadAccelerometer(hgy87, &GY87_accelerometerValues);

            /* Calculate Kalman roll angle */
            Kalman_CalculateAngle(&Kalman_predictionValue_rollAngle, &Kalman_uncertaintyValue_rollAngle, GY87_gyroscopeValues.rotationRateRoll, GY87_accelerometerValues.angleRoll);
            /* Calculate Kalman pitch angle */
            Kalman_CalculateAngle(&Kalman_predictionValue_pitchAngle, &Kalman_uncertaintyValue_pitchAngle, GY87_gyroscopeValues.rotationRatePitch, GY87_accelerometerValues.anglePitch);

            /* Read inputs from radio controller */
            reference_throttle   = FSA8S_ReadChannel(rc_controller, CHANNEL_3);
            reference_rollValue  = FSA8S_ReadChannel(rc_controller, CHANNEL_1);
            reference_pitchValue = FSA8S_ReadChannel(rc_controller, CHANNEL_2);
            reference_yawValue   = FSA8S_ReadChannel(rc_controller, CHANNEL_4);

            /* Adjust and limit throttle input */
            if (CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE < reference_throttle) {
                reference_throttle = CONTROL_SYSTEM_MAXIMUM_INPUT_THROTTLE;
            }

            /* Calculate desired angles by mapping radio controller values to angles */
            reference_rollAngle  = 0.03 * (reference_rollValue - 500);
            reference_pitchAngle = 0.03 * (reference_pitchValue - 500);

            /* Calculate angles errors */
            error_rollAngle  = reference_rollAngle - Kalman_predictionValue_rollAngle;
            error_pitchAngle = reference_pitchAngle - Kalman_predictionValue_pitchAngle;

            /* Calculate PID for roll angle */
            CSM_CalculatePID(&pidOutput_rollAngle, &previousIterm_rollAngle, &previousErrorValue_rollAngle, error_rollAngle, KP_ROLL_ANGLE, KI_ROLL_ANGLE, KD_ROLL_ANGLE);
            /* Calculate PID for pitch angle */
            CSM_CalculatePID(&pidOutput_pitchAngle, &previousIterm_pitchAngle, &previousErrorValue_pitchAngle, error_pitchAngle, KP_PITCH_ANGLE, KI_PITCH_ANGLE, KD_PITCH_ANGLE);

            /* Calculate desired rates */
            desiredValue_rollRate  = pidOutput_rollAngle;
            desiredValue_pitchRate = pidOutput_pitchAngle;
            desiredValue_yawRate   = 0.03 * (reference_yawValue - 500);

            /* Calculate rates errors */
            error_rollRate  = desiredValue_rollRate - GY87_gyroscopeValues.rotationRateRoll;
            error_pitchRate = desiredValue_pitchRate - GY87_gyroscopeValues.rotationRatePitch;
            error_yawRate   = desiredValue_yawRate - GY87_gyroscopeValues.rotationRateYaw;

            /* Calculate PID for roll rate */
            CSM_CalculatePID(&pidOutput_rollRate, &previousIterm_rollRate, &previousErrorValue_rollRate, error_rollRate, KP_ROLL_RATE, KI_ROLL_RATE, KD_ROLL_RATE);
            /* Calculate PID for pitch rate */
            CSM_CalculatePID(&pidOutput_pitchRate, &previousIterm_pitchRate, &previousErrorValue_pitchRate, error_pitchRate, KP_PITCH_RATE, KI_PITCH_RATE, KD_PITCH_RATE);
            /* Calculate PID for yaw rate */
            CSM_CalculatePID(&pidOutput_yawRate, &previousIterm_yawRate, &previousErrorValue_yawRate, error_yawRate, KP_YAW_RATE, KI_YAW_RATE, KD_YAW_RATE);

            /* Calculate motors speed */
            motorSpeed1 = (reference_throttle - pidOutput_rollRate - pidOutput_pitchRate - pidOutput_yawRate) / 10;
            motorSpeed2 = (reference_throttle + pidOutput_rollRate + pidOutput_pitchRate - pidOutput_yawRate) / 10;
            motorSpeed3 = (reference_throttle + pidOutput_rollRate - pidOutput_pitchRate + pidOutput_yawRate) / 10;
            motorSpeed4 = (reference_throttle - pidOutput_rollRate + pidOutput_pitchRate + pidOutput_yawRate) / 10;

            /* Adjust and limit motors maximum speed */
            if (ESC_MAXIMUM_SPEED < motorSpeed1)
                motorSpeed1 = ESC_MAXIMUM_SPEED;
            if (ESC_MAXIMUM_SPEED < motorSpeed2)
                motorSpeed2 = ESC_MAXIMUM_SPEED;
            if (ESC_MAXIMUM_SPEED < motorSpeed3)
                motorSpeed3 = ESC_MAXIMUM_SPEED;
            if (ESC_MAXIMUM_SPEED < motorSpeed4)
                motorSpeed4 = ESC_MAXIMUM_SPEED;

            /* Adjust and limit motors minimum speed */
            if (ESC_MINIMUM_SPEED > motorSpeed1)
                motorSpeed1 = ESC_MINIMUM_SPEED;
            if (ESC_MINIMUM_SPEED > motorSpeed2)
                motorSpeed2 = ESC_MINIMUM_SPEED;
            if (ESC_MINIMUM_SPEED > motorSpeed3)
                motorSpeed3 = ESC_MINIMUM_SPEED;
            if (ESC_MINIMUM_SPEED > motorSpeed4)
                motorSpeed4 = ESC_MINIMUM_SPEED;

            /* Save motors speed */
            ESC_speeds[1] = motorSpeed1;
            ESC_speeds[2] = motorSpeed2;
            ESC_speeds[3] = motorSpeed3;
            ESC_speeds[4] = motorSpeed4;

            /* Set motors speed */
            ESC_SetSpeed(hesc, hesc->esc1, ESC_speeds[4]);
            ESC_SetSpeed(hesc, hesc->esc2, ESC_speeds[2]);
            ESC_SetSpeed(hesc, hesc->esc3, ESC_speeds[3]);
            ESC_SetSpeed(hesc, hesc->esc4, ESC_speeds[1]);
        }

        /* Reset Timer4 flag */
        Timer_Flag_ControlSystem = false;
    }
}
