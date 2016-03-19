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
public class DreamTeamAutonomous extends OpMode {

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
	Servo servoClimbers;
	Servo servoArm;
	Servo servoButton;
	Servo servoTriggerLeft;
	Servo servoTriggerRight;


	double servoRightPos;
	double servoLeftPos;

	/**
	 * Constructor
	 */

	public DreamTeamAutonomous() {
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
		servoArm = hardwareMap.servo.get("servoArm");
		servoClimbers = hardwareMap.servo.get("servoClimbers");
		servoButton = hardwareMap.servo.get("servoButton");
		servoTriggerLeft = hardwareMap.servo.get("servoTriggerLeft");
		servoTriggerRight = hardwareMap.servo.get("servoTriggerRight");
		//Reverse one side to enable the robot to 'walk straight'
		motorFR.setDirection(DcMotor.Direction.REVERSE);
		motorBR.setDirection(DcMotor.Direction.FORWARD);
		motorFL.setDirection(DcMotor.Direction.REVERSE);
		motorBL.setDirection(DcMotor.Direction.FORWARD);
		servoButton.setPosition(1);
		servoLeftPos = 0;
		servoLeft.setPosition(servoLeftPos);
		servoRightPos = 1;
		servoRight.setPosition(servoRightPos);
		servoClimberPos = .35;
		servoClimbers.setPosition(servoClimberPos);
		servoTriggerLeft.setPosition(.5);
		servoTriggerRight.setPosition(.5);
		servoArmPos = .01;
		servoArm.setPosition(servoArmPos);
		cd = new ColorDetection(ColorDetection.COLOR_BLUE);
		resetEncoders();
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

	public enum State {
		TEST, START, TURN, FORWARD, ALIGN, BUTTON, CLIMBERS, CLIMBER_ALIGN_FORWARD, CLIMBER_ALIGN_BACKWARD, MOVE_PARK
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
	int stepCount = 1120;

	int encBL;
	int encFL;
	int encFR;
	int encBR;

	ArrayList<Point> points = new ArrayList();

	long startClimberTime;

	//Dreamteam rotation constant- this function returns the distance to rotate a given number of degrees
	public double rotateDistance(double degrees) {
		return 0.148361 * degrees;
	}

	@Override
	public void loop() {

		//This is the switch for the state machine
		switch(state) {
			case START:
				telemetry.addData("STATE", "START");
				targetPosition  = ((stepCount/12.56)) * 20;//9.8cm
				blDone = motorBL.getCurrentPosition() > targetPosition;
				flDone = motorFL.getCurrentPosition() > targetPosition;
				frDone = motorFR.getCurrentPosition() > targetPosition;
				brDone = motorBR.getCurrentPosition() > targetPosition;
				telemetry.addData("MotorFL", motorBL.getCurrentPosition());

				//Check to see if each motor has finished
				if(blDone) {
					motorBLPow = 0;
				} else {
					motorBLPow = .3;
				}
				if(flDone) {
					motorFLPow = 0;
				} else {
					motorFLPow = .3;
				}
				if(brDone) {
					motorBRPow = 0;
				} else {
					motorBRPow = .3;
				}
				if(frDone) {
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
				targetPosition = rotateDistance(43) * (stepCount/12.56);

				blDone = motorBL.getCurrentPosition() > encBL + targetPosition;
				flDone = motorFL.getCurrentPosition() < encFL - targetPosition;
				frDone = motorFR.getCurrentPosition() < encFR - targetPosition;
				brDone = motorBR.getCurrentPosition() > encBR + targetPosition;

				//Check to see if each motor has finished
				if(blDone) {
					motorBLPow = 0;
				} else {
					motorBLPow = .3;
				}
				if(flDone) {
					motorFLPow = 0;
				} else {
					motorFLPow = -.3;
				}
				if(brDone) {
					motorBRPow = 0;
				} else {
					motorBRPow = .3;
				}
				if(frDone) {
					motorFRPow = 0;
				} else {
					motorFRPow = -.3;
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
				targetPosition  = ((stepCount/12.56)) * 42;//9.8cm
				blDone = motorBL.getCurrentPosition() > encBL + targetPosition;
				flDone = motorFL.getCurrentPosition() > encFL + targetPosition;
				frDone = motorFR.getCurrentPosition() > encFR + targetPosition;
				brDone = motorBR.getCurrentPosition() > encBR + targetPosition;
				telemetry.addData("MotorFL", motorBL.getCurrentPosition());

				//Check to see if each motor has finished
				if(blDone) {
					motorBLPow = 0;
				} else {
					motorBLPow = .3;
				}
				if(flDone) {
					motorFLPow = 0;
				} else {
					motorFLPow = .3;
				}
				if(brDone) {
					motorBRPow = 0;
				} else {
					motorBRPow = .3;
				}
				if(frDone) {
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
				servoButton.setPosition(.45);
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
					if (average.y > 60) {
						motorBLPow = -.04;
						motorFLPow = .04;
						motorBRPow = .04;
						motorFRPow = -.04;
						telemetry.addData("Turn", "Left");
					} else if (average.y < 45) {
						motorBLPow = .04;
						motorFLPow = -.04;
						motorBRPow = -.04;
						motorFRPow = .04;
						telemetry.addData("Turn", "Right");
					} else {
						//Detected and tracked successfully.
						//wait(100);
						//The beacon is now centered
						telemetry.addData("Contour Area", Imgproc.contourArea(cd.getCon()));
						if(Imgproc.contourArea(cd.getCon()) < 3000) {
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
					motorBLPow = -.1;
					motorFLPow = .1;
					motorBRPow = .1;
					motorFRPow = -.1;

				}
				break;
			case BUTTON:
				telemetry.addData("STATE", "BUTTON");
				targetPosition  = ((stepCount/12.56)) * 2.25;
				blDone = motorBL.getCurrentPosition() > encBL + targetPosition;
				flDone = motorFL.getCurrentPosition() < encFL - targetPosition;
				frDone = motorFR.getCurrentPosition() > encFR + targetPosition;
				brDone = motorBR.getCurrentPosition() < encBR - targetPosition;
				telemetry.addData("MotorFL", motorBL.getCurrentPosition());

				//Check to see if each motor has finished
				if(blDone && !buttonSubForward) {
					motorBLPow = 0;
				} else {
					motorBLPow = .1;
				}
				if(flDone && !buttonSubForward) {
					motorFLPow = 0;
				} else {
					motorFLPow = -.1;
				}
				if(brDone && !buttonSubForward) {
					motorBRPow = 0;
				} else {
					motorBRPow = -.1;
				}
				if(frDone && !buttonSubForward) {
					motorFRPow = 0;
				} else {
					motorFRPow = .1;
				}
				boolean tempDone = (blDone && frDone && brDone && flDone);
				//Check to see if all states are done, and if they are, move on to the next state
				if (tempDone || buttonSubForward) {
					if(tempDone) {
						encBL = motorBL.getCurrentPosition();
						encBR = motorBR.getCurrentPosition();
						encFR = motorFR.getCurrentPosition();
						encFL = motorFL.getCurrentPosition();
					}
					buttonSubForward = true;
					targetPosition  = ((stepCount/12.56)) * 4.2;
					blDone = motorBL.getCurrentPosition() > encBL + targetPosition;
					flDone = motorFL.getCurrentPosition() > encFL + targetPosition;
					frDone = motorFR.getCurrentPosition() > encFR + targetPosition;
					brDone = motorBR.getCurrentPosition() > encBR + targetPosition;

					if(blDone) {
						motorBLPow = 0;
					} else {
						motorBLPow = .1;
					}
					if(flDone) {
						motorFLPow = 0;
					} else {
						motorFLPow = .1;
					}
					if(brDone) {
						motorBRPow = 0;
					} else {
						motorBRPow = .1;
					}
					if(frDone) {
						motorFRPow = 0;
					} else {
						motorFRPow = .1;
					}

					tempDone = (blDone && frDone && brDone && flDone);
					if (tempDone) {
						state = State.CLIMBER_ALIGN_BACKWARD;
						nextState = State.CLIMBER_ALIGN_FORWARD;
						encBL = motorBL.getCurrentPosition();
						encBR = motorBR.getCurrentPosition();
						encFR = motorFR.getCurrentPosition();
						encFL = motorFL.getCurrentPosition();
					}

				}
				break;
			case CLIMBERS:
				telemetry.addData("STATE", "CLIMBERS");
				if(servoArmPos < .9) {
					servoArmPos += climberServoInc;
				}
				if (servoClimberPos < .9) {
					servoClimberPos += climberServoInc;
				}
				boolean done = !(servoClimberPos < .8 && servoArmPos < .7);
				servoArm.setPosition(servoArmPos);
				servoClimbers.setPosition(servoClimberPos);
				if(done) {
					servoClimbers.setPosition(0);
					servoArm.setPosition(.7);
					state = State.MOVE_PARK;
				}
				break;
			case CLIMBER_ALIGN_BACKWARD:
				telemetry.addData("STATE", "CLIMBER_ALIGN_BACKWARD");
				targetPosition  = ((stepCount/12.56)) * 1.5;
				blDone = motorBL.getCurrentPosition() < encBL - targetPosition;
				flDone = motorFL.getCurrentPosition() < encFL - targetPosition;
				frDone = motorFR.getCurrentPosition() < encFR - targetPosition;
				brDone = motorBR.getCurrentPosition() < encBR - targetPosition;

				if(blDone) {
					motorBLPow = 0;
				} else {
					motorBLPow = -.1;
				}
				if(flDone) {
					motorFLPow = 0;
				} else {
					motorFLPow = -.1;
				}
				if(brDone) {
					motorBRPow = 0;
				} else {
					motorBRPow = -.1;
				}
				if(frDone) {
					motorFRPow = 0;
				} else {
					motorFRPow = -.1;
				}


				tempDone = (blDone && frDone && brDone && flDone);
				if (tempDone) {
					state = nextState;
					nextState = State.CLIMBERS;
					encBL = motorBL.getCurrentPosition();
					encBR = motorBR.getCurrentPosition();
					encFR = motorFR.getCurrentPosition();
					encFL = motorFL.getCurrentPosition();
				}
				break;
			case CLIMBER_ALIGN_FORWARD:
				telemetry.addData("STATE", "CLIMBER_ALIGN_FORWARD");
				targetPosition  = ((stepCount/12.56)) * 6.5;
				servoButton.setPosition(1);
				blDone = motorBL.getCurrentPosition() > encBL + targetPosition;
				flDone = motorFL.getCurrentPosition() > encFL + targetPosition;
				frDone = motorFR.getCurrentPosition() > encFR + targetPosition;
				brDone = motorBR.getCurrentPosition() > encBR + targetPosition;

				if(blDone) {
					motorBLPow = 0;
				} else {
					motorBLPow = .1;
				}
				if(flDone) {
					motorFLPow = 0;
				} else {
					motorFLPow = .1;
				}
				if(brDone) {
					motorBRPow = 0;
				} else {
					motorBRPow = .1;
				}
				if(frDone) {
					motorFRPow = 0;
				} else {
					motorFRPow = .1;
				}


				tempDone = (blDone && frDone && brDone && flDone);
				if (tempDone) {
					state = nextState;
					nextState = State.TEST;
					encBL = motorBL.getCurrentPosition();
					encBR = motorBR.getCurrentPosition();
					encFR = motorFR.getCurrentPosition();
					encFL = motorFL.getCurrentPosition();
					startClimberTime = System.currentTimeMillis();
				}
				break;
			case MOVE_PARK:
				targetPosition  = ((stepCount/12.56)) * 18;
				blDone = motorBL.getCurrentPosition() > encBL + targetPosition;
				flDone = motorFL.getCurrentPosition() < encFL - targetPosition;
				frDone = motorFR.getCurrentPosition() > encFR + targetPosition;
				brDone = motorBR.getCurrentPosition() < encBR - targetPosition;
				telemetry.addData("MotorFL", motorBL.getCurrentPosition());
				/*
					motorBLPow = .1;
					motorFLPow = -.1;
					motorBRPow = -.1;
					motorFRPow = .1;
				*/

				//Check to see if each motor has finished
				if(blDone) {
					motorBLPow = 0;
				} else {
					motorBLPow = .1;
				}
				if(flDone) {
					motorFLPow = 0;
				} else {
					motorFLPow = -.1;
				}
				if(brDone) {
					motorBRPow = 0;
				} else {
					motorBRPow = -.1;
				}
				if(frDone) {
					motorFRPow = 0;
				} else {
					motorFRPow = .1;
				}
				tempDone = (blDone && frDone && brDone && flDone);
				//Check to see if all states are done, and if they are, move on to the next state
				if (tempDone) {
					state = State.TEST;
					servoClimbers.setPosition(.35);
					servoArm.setPosition(.01);
					encBL = motorBL.getCurrentPosition();
					encBR = motorBR.getCurrentPosition();
					encFR = motorFR.getCurrentPosition();
					encFL = motorFL.getCurrentPosition();
				}
				break;
			default:
				break;
		}


		//Always execute after every loop- this will update motor speeds.
		motorBR.setPower(motorBRPow);
		motorFL.setPower(motorFLPow);
		motorFR.setPower(motorFRPow);
		motorBL.setPower(motorBLPow);

		//Servos
		servoLeft.setPosition(servoLeftPos);
		servoRight.setPosition(servoRightPos);
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
		resetEncoders();
		state = State.START;
		nextState = State.TURN;
	}
}