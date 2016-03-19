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

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class DreamTeamAutonomousRedClose extends OpMode {

    /*
    Main teleop file of the 2015-16 dream team robot
    Coded by the 31337 Matthew Raneri
     */

	ColorDetection cd;

	//Define motors
	DcMotor motorFR;
	DcMotor motorFL;
	DcMotor motorBR;
	DcMotor motorBL;

	//Define servos
	Servo servoLeft;
	Servo servoRight;
	Servo servoPan;
	Servo servoClimbers;
	Servo servoArm;

	//It isn't good to define variables in a loop- can potentially give java a little bit of a hard time doing
	//garbage clean up.
	//
	double servoPanPos;
	double servoRightPos;
	double servoLeftPos;
	double servoIncrement = 0.003;

	//Motor power and RPM values
	double powerBR_FL;
	double powerFR_BL;

	double Joy1X;
	double Joy1Y;

	/**
	 * Constructor
	 */

	public DreamTeamAutonomousRedClose() {
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

		//motorCollection = hardwareMap.dcMotor.get("motorCollection");

		servoLeft = hardwareMap.servo.get("servoLeft");
		servoRight = hardwareMap.servo.get("servoRight");
		servoPan = hardwareMap.servo.get("servoPan");
		servoArm = hardwareMap.servo.get("servoArm");
		servoClimbers = hardwareMap.servo.get("servoClimbers");

		//Reverse one side to enable the robot to 'walk straight'
		motorFR.setDirection(DcMotor.Direction.REVERSE);
		motorBR.setDirection(DcMotor.Direction.FORWARD);
		motorFL.setDirection(DcMotor.Direction.REVERSE);
		motorBL.setDirection(DcMotor.Direction.FORWARD);
		servoPanPos = .48;
		servoPan.setPosition(servoPanPos);
		servoLeftPos = 0;
		servoLeft.setPosition(servoLeftPos);
		servoRightPos = 1;
		servoRight.setPosition(servoRightPos);
		servoClimberPos = .45;
		servoClimbers.setPosition(servoClimberPos);
		servoArmPos = .2;
		servoArm.setPosition(servoArmPos);
		cd = new ColorDetection(ColorDetection.COLOR_BLUE);
		resetEncoders();
		delayTime = System.currentTimeMillis();
	}


	public void resetEncoders() {
		motorFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		motorFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		motorBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		motorBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
	}


	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */

	double motorBRPow;
	double motorFLPow;
	double motorFRPow;
	double motorBLPow;

	double climberServoInc = 0.0025;
	double servoClimberPos;
	double servoArmPos;

	boolean debug = false;

	public enum State {
		TEST, START, TURN, FORWARD, ALIGN, BUTTON, CLIMBERS
	}

	State state = State.START;
	State nextState = State.TURN;

	//Target position of motors to complete state
	double targetPosition;

	//Booleans to check state of each motor if it has completed its task
	boolean blDone;
	boolean flDone;
	boolean frDone;
	boolean brDone;

	boolean buttonSubForward = false;
	boolean buttonSubBack = false;

	int turnCount; //This represents the amount of times we have completed the turn state.

	//stepcount- resolution of encoder per one rotation of the wheel
	int stepCount = 1120;

	int encBL;
	int encFL;
	int encFR;
	int encBR;

	ArrayList<Point> points = new ArrayList();

	long startClimberTime;
	long delayTime;
	//Dreamteam rotation constant- this function returns the distance to rotate a given number of degrees
	public double rotateDistance(double degrees) {
		return 0.148361 * degrees;
	}

	@Override
	public void loop() {
		//Joy1Y = gamepad1.left_stick_y;
		//Joy1X = gamepad1.left_stick_x;
		// clip the right/left values so that the values never exceed +/- 1

		//Joy1Y = Range.clip(Joy1Y, -1, 1);
		//Joy1X = Range.clip(Joy1X, -1, 1);
		//write to motors
		//Keep in mind that the power for opposite motors is the same
		//powerBR_FL = calculateMotorPower(Joy1X, Joy1Y);
		//powerFR_BL = calculateMotorPower(Joy1X, -Joy1Y);

		//double offset = scaleInput(gamepad1.right_stick_x);

		//This is the switch for the state machine
		if(delayTime + 5000 < System.currentTimeMillis()) {
			switch (state) {
				case START:
					telemetry.addData("STATE", "START");
					targetPosition = ((stepCount / 12.56)) * 20;//9.8cm
					blDone = motorBL.getCurrentPosition() > targetPosition;
					flDone = motorFL.getCurrentPosition() > targetPosition;
					frDone = motorFR.getCurrentPosition() > targetPosition;
					brDone = motorBR.getCurrentPosition() > targetPosition;
					telemetry.addData("MotorFL", motorBL.getCurrentPosition());

					//Check to see if each motor has finished
					if (blDone) {
						motorBLPow = 0;
					} else {
						motorBLPow = .3;
					}
					if (flDone) {
						motorFLPow = 0;
					} else {
						motorFLPow = .3;
					}
					if (brDone) {
						motorBRPow = 0;
					} else {
						motorBRPow = .3;
					}
					if (frDone) {
						motorFRPow = 0;
					} else {
						motorFRPow = .3;
					}
					//Check to see if all states are done, and if they are, move on to the next state
					if (blDone && frDone && brDone && flDone) {
						state = nextState;
						//After the following state finishes, what state should should go next
						nextState = State.FORWARD;
						encBL = motorBL.getCurrentPosition();
						encBR = motorBR.getCurrentPosition();
						encFR = motorFR.getCurrentPosition();
						encFL = motorFL.getCurrentPosition();
					}
					break;
				case TURN:
					telemetry.addData("STATE", "TURN");
					targetPosition = rotateDistance(43) * (stepCount / 12.56);

					blDone = motorBL.getCurrentPosition() < encBL - targetPosition;
					flDone = motorFL.getCurrentPosition() > encFL + targetPosition;
					frDone = motorFR.getCurrentPosition() > encFR + targetPosition;
					brDone = motorBR.getCurrentPosition() < encBR - targetPosition;

					//Check to see if each motor has finished
					if (blDone) {
						motorBLPow = 0;
					} else {
						motorBLPow = -.3;
					}
					if (flDone) {
						motorFLPow = 0;
					} else {
						motorFLPow = .3;
					}
					if (brDone) {
						motorBRPow = 0;
					} else {
						motorBRPow = -.3;
					}
					if (frDone) {
						motorFRPow = 0;
					} else {
						motorFRPow = .3;
					}
					//Check to see if all states are done, and if they are, move on to the next state
					if (blDone && frDone && brDone && flDone) {
						state = nextState;
						//After the following state finishes, what state should should go next
						nextState = State.TURN;
						encBL = motorBL.getCurrentPosition();
						encBR = motorBR.getCurrentPosition();
						encFR = motorFR.getCurrentPosition();
						encFL = motorFL.getCurrentPosition();

					}

					break;

				case FORWARD:
					telemetry.addData("STATE", "FORWARD");
					targetPosition = ((stepCount / 12.56)) * 55;//9.8cm
					blDone = motorBL.getCurrentPosition() > encBL + targetPosition;
					flDone = motorFL.getCurrentPosition() > encFL + targetPosition;
					frDone = motorFR.getCurrentPosition() > encFR + targetPosition;
					brDone = motorBR.getCurrentPosition() > encBR + targetPosition;
					telemetry.addData("MotorFL", motorBL.getCurrentPosition());

					//Check to see if each motor has finished
					if (blDone) {
						motorBLPow = 0;
					} else {
						motorBLPow = .3;
					}
					if (flDone) {
						motorFLPow = 0;
					} else {
						motorFLPow = .3;
					}
					if (brDone) {
						motorBRPow = 0;
					} else {
						motorBRPow = .3;
					}
					if (frDone) {
						motorFRPow = 0;
					} else {
						motorFRPow = .3;
					}
					//Check to see if all states are done, and if they are, move on to the next state
					if (blDone && frDone && brDone && flDone) {
						state = nextState;
						//After the following state finishes, what state should should go next
						nextState = State.ALIGN;
						encBL = motorBL.getCurrentPosition();
						encBR = motorBR.getCurrentPosition();
						encFR = motorFR.getCurrentPosition();
						encFL = motorFL.getCurrentPosition();
					}
					break;

				case ALIGN:
					telemetry.addData("STATE", "ALIGN");
					cd.caploop();
					if (cd.getCon() != null && Imgproc.contourArea(cd.getCon()) > 100) {
						Point center;
						//I want to store the centroid of the mask in the point 'center'
						//I also have access to the contour I want to use above
						//but however I try to compute the moments, I get a crash
						Rect rect = Imgproc.boundingRect(cd.getCon());
						center = new Point((rect.x + rect.width) / 2, (rect.y + rect.height) / 2);
						points.add(center);
						if (points.size() > 3) {
							points.remove(0);
						}

						Point average = new Point();
						for (Point p : points) {
							average.x = average.x + p.x;
							average.y = average.y + p.y;
						}

						average.y = (average.y / points.size());
						average.x = (average.x / points.size());


						telemetry.addData("X, Y", average.x + " " + average.y);
						if (average.y < 45) {
							motorBLPow = .04;
							motorFLPow = -.04;
							motorBRPow = -.04;
							motorFRPow = .04;
							telemetry.addData("Turn", "Left");
						} else if (average.y > 60) {
							motorBLPow = -.04;
							motorFLPow = .04;
							motorBRPow = .04;
							motorFRPow = -.04;
							telemetry.addData("Turn", "Right");
						} else {
							//Detected and tracked successfully.
							//wait(100);
							//The beacon is now centered
							telemetry.addData("Contour Area", Imgproc.contourArea(cd.getCon()));
							if (Imgproc.contourArea(cd.getCon()) < 2000) {
								motorBLPow = 0.07;
								motorFLPow = 0.07;
								motorBRPow = 0.07;
								motorFRPow = 0.07;
							} else {
								motorBLPow = 0;
								motorFLPow = 0;
								motorBRPow = 0;
								motorFRPow = 0;

								encBL = motorBL.getCurrentPosition();
								encBR = motorBR.getCurrentPosition();
								encFR = motorFR.getCurrentPosition();
								encFL = motorFL.getCurrentPosition();

								state = State.BUTTON;
							}
						}

					} else {
						points.clear();
						//
						motorBLPow = -.05;
						motorFLPow = .05;
						motorBRPow = .05;
						motorFRPow = -.05;

					}
					break;
				case BUTTON:
					telemetry.addData("STATE", "BUTTON");
					targetPosition = (stepCount / 12.56) * 8;//9.8cm


				/*

						motorBLPow = .04;
						motorFLPow = -.04;
						motorBRPow = -.04;
						motorFRPow = .04;
				 */


					blDone = motorBL.getCurrentPosition() < encBL - targetPosition;
					flDone = motorFL.getCurrentPosition() > encFL + targetPosition;
					frDone = motorFR.getCurrentPosition() < encFR - targetPosition;
					brDone = motorBR.getCurrentPosition() > encBR + targetPosition;
					telemetry.addData("MotorFL", motorBL.getCurrentPosition());

					//Check to see if each motor has finished
					if (blDone && !buttonSubForward) {
						motorBLPow = 0;
					} else {
						motorBLPow = -.1;
					}
					if (flDone && !buttonSubForward) {
						motorFLPow = 0;
					} else {
						motorFLPow = .1;
					}
					if (brDone && !buttonSubForward) {
						motorBRPow = 0;
					} else {
						motorBRPow = .1;
					}
					if (frDone && !buttonSubForward) {
						motorFRPow = 0;
					} else {
						motorFRPow = -.1;
					}
					boolean tempDone = (blDone && frDone && brDone && flDone);
					//Check to see if all states are done, and if they are, move on to the next state
					if (tempDone || buttonSubForward) {
						if (tempDone) {
							encBL = motorBL.getCurrentPosition();
							encBR = motorBR.getCurrentPosition();
							encFR = motorFR.getCurrentPosition();
							encFL = motorFL.getCurrentPosition();
						}
						buttonSubForward = true;
						targetPosition = ((stepCount / 12.56)) * 8;
						blDone = motorBL.getCurrentPosition() > encBL + targetPosition;
						flDone = motorFL.getCurrentPosition() > encFL + targetPosition;
						frDone = motorFR.getCurrentPosition() > encFR + targetPosition;
						brDone = motorBR.getCurrentPosition() > encBR + targetPosition;

						if (blDone) {
							motorBLPow = 0;
						} else {
							motorBLPow = .1;
						}
						if (flDone) {
							motorFLPow = 0;
						} else {
							motorFLPow = .1;
						}
						if (brDone) {
							motorBRPow = 0;
						} else {
							motorBRPow = .1;
						}
						if (frDone) {
							motorFRPow = 0;
						} else {
							motorFRPow = .1;
						}

						tempDone = (blDone && frDone && brDone && flDone);
						if (tempDone) {
							//
							servoArm.setPosition(.8);
							servoClimbers.setPosition(1);
							//
							state = State.CLIMBERS;
							encBL = motorBL.getCurrentPosition();
							encBR = motorBR.getCurrentPosition();
							encFR = motorFR.getCurrentPosition();
							encFL = motorFL.getCurrentPosition();
							startClimberTime = System.currentTimeMillis();
						}

					}
					break;
				case CLIMBERS:
					telemetry.addData("STATE", "CLIMBERS");
					if (servoArmPos < .89) {
						servoArmPos += climberServoInc;
					}
					if (servoClimberPos > .05) {
						servoClimberPos -= climberServoInc / 3;
					}
					boolean done = !(servoClimberPos > .05 && servoArmPos < .850);
					servoArm.setPosition(servoArmPos);
					servoClimbers.setPosition(servoClimberPos);
					if (done)
						state = State.TEST;
					break;
				default:
					break;
			}
		}


		//Always execute after every loop- this will update motor speeds.
		motorBR.setPower(motorBRPow);
		motorFL.setPower(motorFLPow);
		motorFR.setPower(motorFRPow);
		motorBL.setPower(motorBLPow);

		//Servos
		servoLeft.setPosition(servoLeftPos);
		servoRight.setPosition(servoRightPos);
		servoPan.setPosition(servoPanPos);

		//Debug output
		if(debug) {

			//Current State
			telemetry.addData("DEBUG", "State: " + state.name());

			//Motors
			telemetry.addData("DEBUG", "BR: " + motorBRPow);
			telemetry.addData("DEBUG", "FR: " + motorFRPow);
			telemetry.addData("DEBUG", "BL: " + motorBLPow);
			telemetry.addData("DEBUG", "FL: " + motorFLPow);

			//Servos
			telemetry.addData("DEBUG", "Right: " + motorFLPow);
			telemetry.addData("DEBUG", "Left: " + motorFLPow);
			telemetry.addData("DEBUG", "Pan: " + motorFLPow);
			telemetry.addData("DEBUG", "Button: Disabled");// + motorFLPow);
			//telemetry.addData("DEBUG", "Climbers: Disabled");

		}
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
		encBL = motorBL.getCurrentPosition();
		encFL = motorFL.getCurrentPosition();
		encBR = motorBR.getCurrentPosition();
		encFR = motorFR.getCurrentPosition();
		resetEncoders();
		state = State.START;
		nextState = State.TURN;
	}


	//The motor power function is the same for diagonals but rotated 90 degrees for the opposites
	//Taking the inverse creates limits, doesn't work well with motor control
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