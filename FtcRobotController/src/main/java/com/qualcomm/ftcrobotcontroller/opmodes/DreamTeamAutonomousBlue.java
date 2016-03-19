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
public class DreamTeamAutonomousBlue extends OpMode {

    /*
    Automonous file of the 2015-16 dream team robot
    Coded by the 31337 Matthew Raneri
     */


	//Define motors
	DcMotor motorFR;
	DcMotor motorFL;
	DcMotor motorBR;
	DcMotor motorBL;
	DcMotor motorLift;
	DcMotor motorTilt;
	DcMotor motorCollection;

	//Define servos
	Servo servoLeft;
	Servo servoRight;
	Servo servoPan;
	Servo servoTilt;

	//It isn't good to define variables in a loop- can potentially give java a little bit of a hard time doing
	//garbage clean up.
	//
	double servoTiltPos;
	double servoPanPos;
	double servoRightPos;
	double servoLeftPos;
	double servoIncrement = 0.017;

	boolean didExecute = false;


	/**
	 * Constructor
	 */

	public DreamTeamAutonomousBlue() {
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
		motorLift = hardwareMap.dcMotor.get("motorLift");
		motorTilt = hardwareMap.dcMotor.get("motorTilt");
		motorTilt.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		motorTilt.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

		motorTilt.setMode(DcMotorController.RunMode.RESET_ENCODERS);

		motorCollection = hardwareMap.dcMotor.get("motorCollection");

		servoLeft = hardwareMap.servo.get("servoLeft");
		servoRight = hardwareMap.servo.get("servoRight");
		servoTilt = hardwareMap.servo.get("servoTilt");
		servoPan = hardwareMap.servo.get("servoPan");

		//Reverse one side to enable the robot to 'walk straight'
		motorFR.setDirection(DcMotor.Direction.REVERSE);
		motorFL.setDirection(DcMotor.Direction.REVERSE);
		motorLift.setDirection(DcMotor.Direction.REVERSE);
		servoPanPos = .544;
		servoTiltPos = .153;
		motorLift.setPower(0.2);
		sleep(800);
		motorLift.setPower(0);
		servoPan.setPosition(servoPanPos);
		servoTilt.setPosition(servoTiltPos);

		//cd = new ColorDetection(ColorDetection.COLOR_RED);
		//cd.setupCamera();
	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		if(didExecute) {
			return;
		}
		//Must divide motion by two
		//1440 ticks per rotation
		//12.56 inches per rotation
		//must go 3 feet, 32 inches
		//must make 90 degree turn
		//must go another 3 feet, 32 inches
		//1440 * (32*2)
		motorBL.setPower(0.5);
		motorFL.setPower(0.5);
		motorBR.setPower(0.5);
		motorFR.setPower(0.5);
		boolean bl = motorBL.getCurrentPosition() < 1440 * (32 / 12.56);
		boolean br = motorBR.getCurrentPosition() < 1440 * (32 / 12.56);
		boolean fl = motorFL.getCurrentPosition() < 1440 * (32 / 12.56);
		boolean fr = motorFR.getCurrentPosition() < 1440 * (32 / 12.56);

		while(bl && br && fl && fr) {
			bl = motorBL.getCurrentPosition() < 1440 * (32 / 12.56);
			br = motorBR.getCurrentPosition() < 1440 * (32 / 12.56);
			fl = motorFL.getCurrentPosition() < 1440 * (32 / 12.56);
			fr = motorFR.getCurrentPosition() < 1440 * (32 / 12.56);
		}
		motorBL.setPower(0);
		motorFL.setPower(0);
		motorBR.setPower(0);
		motorFR.setPower(0);
		sleep(500);
		motorBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);

		//Do turn
		//Diagonal of the square equals the diameter of turning circle
		//Diameter of turning circle = 25.45 inches
		//Divide diameter by 4
		//multiply 4 by 1440
		//turn until encoders hit 1440

		bl = motorBL.getCurrentPosition() < 1440 * (25.45 / 4);
		br = motorBR.getCurrentPosition() < 1440 * (25.45 / 4);
		fl = motorFL.getCurrentPosition() < 1440 * (25.45 / 4);
		fr = motorFR.getCurrentPosition() < 1440 * (25.45 / 4);

		motorBL.setPower(-0.5);
		motorFL.setPower(-0.5);
		motorBR.setPower(0.5);
		motorFR.setPower(0.5);

		while(bl && br && fl && fr) {
			bl = motorBL.getCurrentPosition() < 1440 * (32 / 12.56);
			br = motorBR.getCurrentPosition() < 1440 * (32 / 12.56);
			fl = motorFL.getCurrentPosition() < 1440 * (32 / 12.56);
			fr = motorFR.getCurrentPosition() < 1440 * (32 / 12.56);
		}

		motorBL.setPower(0);
		motorFL.setPower(0);
		motorBR.setPower(0);
		motorFR.setPower(0);
		sleep(500);
		motorBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);

		bl = motorBL.getCurrentPosition() < 1440 * (32 / 12.56);
		br = motorBR.getCurrentPosition() < 1440 * (32 / 12.56);
		fl = motorFL.getCurrentPosition() < 1440 * (32 / 12.56);
		fr = motorFR.getCurrentPosition() < 1440 * (32 / 12.56);


		while(bl && br && fl && fr) {
			bl = motorBL.getCurrentPosition() < 1440 * (32 / 12.56);
			br = motorBR.getCurrentPosition() < 1440 * (32 / 12.56);
			fl = motorFL.getCurrentPosition() < 1440 * (32 / 12.56);
			fr = motorFR.getCurrentPosition() < 1440 * (32 / 12.56);
		}

		motorBL.setPower(0);
		motorFL.setPower(0);
		motorBR.setPower(0);
		motorFR.setPower(0);
		sleep(500);
		didExecute = true;

	}

	public void sleep(long time) {
		try {
			Thread.sleep(time);
		} catch(Exception e) {

		}
	}

	public void handleServoMovement() {
		if(gamepad2.left_stick_x < 0 && servoLeftPos + servoIncrement < (1 - servoIncrement)) {
			servoLeftPos += servoIncrement;
		} else if(gamepad2.left_stick_x > 0 && servoLeftPos - servoIncrement > servoIncrement) {
			servoLeftPos -= servoIncrement;
		}
		servoLeft.setPosition(servoLeftPos);

		if(gamepad2.right_stick_x < 0 && servoRightPos + servoIncrement < (1 - servoIncrement)) {
			servoRightPos += servoIncrement;
		} else if(gamepad2.right_stick_x > 0 && servoRightPos - servoIncrement > servoIncrement) {
			servoRightPos -= servoIncrement;
		}
		servoRight.setPosition(servoRightPos);
		telemetry.addData("Right","" + servoRightPos);
		telemetry.addData("Left","" + servoLeftPos);
		if(gamepad2.right_bumper) {
			servoTiltPos += servoIncrement;
		}
		if(gamepad2.right_trigger == 1) {
			servoTiltPos -= servoIncrement;
		}

		if(gamepad2.left_bumper) {
			servoPanPos += servoIncrement;
		}
		if(gamepad2.left_trigger == 1) {
			servoPanPos -= servoIncrement;
		}

		servoTiltPos = Range.clip(servoTiltPos, 0, 1);
		servoPanPos = Range.clip(servoPanPos, 0, 1);
		servoTilt.setPosition(servoTiltPos);
		servoPan.setPosition(servoPanPos);
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
