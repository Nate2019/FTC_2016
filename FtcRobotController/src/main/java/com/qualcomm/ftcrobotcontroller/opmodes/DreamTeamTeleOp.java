/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class DreamTeamTeleOp extends OpMode {

    /*
    Main teleop file of the 2015-16 dream team robot
    Coded by the 31337 Matthew Raneri
     */

    //Define motors
    DcMotor motorFR;
	DcMotor motorFL;
	DcMotor motorBR;
	DcMotor motorBL;
	DcMotor motorLock;

    //Define servos
    Servo servoLeft;
    Servo servoRight;
    Servo servoArm;
    Servo servoTriggerRight;
    Servo servoTriggerLeft;
	Servo servoClimbers;
	Servo servoButton;

    //It isn't good to define variables in a loop- can potentially give java a little bit of a hard time doing
    //garbage clean up.
    //
    double servoRightPos;
    double servoLeftPos;
    double servoIncrement = 0.05;

	double orientationValue = 1;

    //Motor power and RPM values
	double powerBR_FL;
    double powerFR_BL;

    double Joy1X;
    double Joy1Y;

    /**
     * Constructor
     */

	public DreamTeamTeleOp() {
	}

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */



        //Link classes to the configuration file names of motors
        motorFR = hardwareMap.dcMotor.get("motorFR");
		motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFL = hardwareMap.dcMotor.get("motorFL");
		motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR = hardwareMap.dcMotor.get("motorBR");
		motorBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL = hardwareMap.dcMotor.get("motorBL");
		motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		motorLock = hardwareMap.dcMotor.get("motorLock");

		//motorCollection = hardwareMap.dcMotor.get("motorCollection");

		servoLeft = hardwareMap.servo.get("servoLeft");
        servoRight = hardwareMap.servo.get("servoRight");
        servoArm = hardwareMap.servo.get("servoArm");
        servoTriggerLeft = hardwareMap.servo.get("servoTriggerLeft");
        servoTriggerRight = hardwareMap.servo.get("servoTriggerRight");
		servoTriggerLeft.setPosition(.5);
		servoTriggerRight.setPosition(.5);
        servoArm.setPosition(0);
		servoClimbers = hardwareMap.servo.get("servoClimbers");
		servoClimbers.setPosition(.35);
		servoButton = hardwareMap.servo.get("servoButton");
		servoButton.setPosition(1);
        //Reverse one side to enable the robot to 'walk straight'
		motorFR.setDirection(DcMotor.Direction.REVERSE);
		motorBR.setDirection(DcMotor.Direction.FORWARD);
		motorFL.setDirection(DcMotor.Direction.REVERSE);
		motorBL.setDirection(DcMotor.Direction.FORWARD);
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
	public void loop() {
		Joy1Y = gamepad1.left_stick_y;
		Joy1X = gamepad1.left_stick_x;
		// clip the right/left values so that the values never exceed +/- 1

		Joy1Y = Range.clip(Joy1Y, -1, 1);
		Joy1X = Range.clip(Joy1X, -1, 1);
		//write to motors
		//Keep in mind that the power for opposite motors is the same
		powerBR_FL = -calculateMotorPower(-Joy1X, Joy1Y);
		powerFR_BL = calculateMotorPower(-Joy1X, -Joy1Y);

		double offset = scaleInput(gamepad1.right_stick_x);

		double motorBRPow = powerBR_FL + (offset * orientationValue);
		double motorFLPow = powerBR_FL - (offset * orientationValue);
		double motorFRPow = powerFR_BL - (offset * orientationValue);
		double motorBLPow = powerFR_BL + (offset * orientationValue);

		motorBRPow = Range.clip(motorBRPow, -1, 1);
		motorFLPow = Range.clip(motorFLPow, -1, 1);
		motorFRPow = Range.clip(motorFRPow, -1, 1);
		motorBLPow = Range.clip(motorBLPow, -1, 1);
		telemetry.addData("RightPower","" + motorBRPow);
		telemetry.addData("LeftPower", "" + motorFLPow);

		motorBR.setPower(motorBRPow * orientationValue);
		motorFL.setPower(motorFLPow * orientationValue);
		motorFR.setPower(motorFRPow * orientationValue);
		motorBL.setPower(motorBLPow * orientationValue);

		if(gamepad1.a) {
			orientationValue = 1;
		} else if(gamepad1.y) {
			orientationValue = -1;
		}

		//Servos
		handleServoMovement();
	}

    public void handleServoMovement() {
		servoLeftPos = servoLeft.getPosition();
		servoRightPos = servoRight.getPosition();
		if(gamepad2.left_stick_x < 0) {
			servoLeftPos += 0.1;
		} else if(gamepad2.left_stick_x > 0) {
			servoLeftPos -= 0.1;
		}
		servoLeft.setPosition(Range.clip(servoLeftPos, 0, 1));

		if(gamepad2.right_stick_x < 0) {
			servoRightPos += 0.1;
		} else if(gamepad2.right_stick_x > 0) {
			servoRightPos -= 0.1;
		}
		servoRight.setPosition(Range.clip(servoRightPos, 0, 1));

        if(gamepad2.right_bumper) {
			servoTriggerRight.setPosition(1);
		} else if(gamepad2.right_trigger > .5) {
			servoTriggerRight.setPosition(0);
		} else {
			servoTriggerRight.setPosition(.5);
		}

		if(gamepad2.left_bumper) {
			servoTriggerLeft.setPosition(1);
		} else if(gamepad2.left_trigger > .5) {
			servoTriggerLeft.setPosition(0);
		} else {
			servoTriggerLeft.setPosition(.5);
		}


		if(gamepad1.dpad_up) {
			motorLock.setPower(-1);
		} else if(gamepad1.dpad_down) {
			motorLock.setPower(1);
		} else {
			motorLock.setPower(0);
		}

		if(gamepad2.dpad_up) {
			servoArm.setPosition(Range.clip(servoArm.getPosition() + servoIncrement, 0, 1));
			servoClimbers.setPosition(Range.clip(servoClimbers.getPosition() + servoIncrement, 0, 1));
		} else if(gamepad2.dpad_down) {
			servoArm.setPosition(Range.clip(servoArm.getPosition() - servoIncrement, 0, 1));
			servoClimbers.setPosition(Range.clip(servoClimbers.getPosition() - servoIncrement, 0, 1));
		}
		if(gamepad2.dpad_left) {
			servoClimbers.setPosition(Range.clip(servoClimbers.getPosition() - servoIncrement, 0, 1));
		} else if(gamepad2.dpad_right) {
			servoClimbers.setPosition(Range.clip(servoClimbers.getPosition() + servoIncrement, 0, 1));
		}

		telemetry.addData("Right","" + servoRightPos);
		telemetry.addData("Left","" + servoLeftPos);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
		motorBL.setPower(0);
		motorBR.setPower(0);
		motorFL.setPower(0);
		motorFR.setPower(0);
		//pid.setIsRunning(false);
		//rt.setIsRunning(false);
    }


    //The motor power function is the same for diagonals but rotated 90 degrees for the opposites
    //Taking the inverse creates limits, doesn't work well with motor control- difficult to programmatically
    //Destroy the limits
    public double calculateMotorPower(double x, double y) {
        //The motor power is function of the slope of the line
        //we will first get the slope of the line- the line always passes through the origin
        //Check to see if the function is undefined or has a slope of zero, and just return simple values
        if(x == 0) return y;
        if(y == 0) return -x;
        //The point at which the motors will be equal to zero
        double midpoint = .7;
        //Scaling factor
        double scalingFactor = 10;
        //Calculate slope
        double slope = y/x;
		//Scale down values to enable better control
        double power = (slope - (slope/scalingFactor) - midpoint);
		power = power;
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        //Quadrant 4 and 1 calculations- inversions
        if((x > 0 && y > 0) || (x > 0 && y < 0)) {
            return power;
        }
        //Quadrant 3 and 2 calculations- inversions
        if((x < 0 && y > 0) || (x < 0 && y < 0)) {
            return -power;
        }
        return power;
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
