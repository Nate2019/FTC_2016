package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Matthew on 11/12/2015.
 */
public class RateThread extends Thread{
	private double[] lastTimes;
	private int[] measuredVals;
	private int[] overlaps;
	private DcMotor[] motors;
	private boolean isRunning;
	public RateThread() {
		isRunning = true;
		lastTimes = new double[4];
		measuredVals = new int[4];
		overlaps = new int[4];
		//motors = new DcMotor[4];
		//motors[0] = DreamTeamTeleOp.motorBR;
		//motors[1] = DreamTeamTeleOp.motorFR;
		//motors[2] = DreamTeamTeleOp.motorFL;
		//motors[3] = DreamTeamTeleOp.motorBL;
	}

	@Override
	public void run() {
		if(isRunning) {
			for(int i = 0; i < lastTimes.length; i++) {
				lastTimes[i] = System.currentTimeMillis();
				measuredVals[i] = motors[i].getCurrentPosition();
				if(measuredVals[i] >= 1440) {
					motors[i].setMode(DcMotorController.RunMode.RESET_ENCODERS);
					overlaps[i] += 1;
					measuredVals[i] = 0;
				} else if(measuredVals[i] <= 1440) {
					motors[i].setMode(DcMotorController.RunMode.RESET_ENCODERS);
					overlaps[i] -= 1;
					measuredVals[i] = 0;
				}
			}
			pause(5);
		}
	}

	public double getRate(double currentEnc, long newTime, int motorNum) {
		return (measuredVals[motorNum] + (overlaps[motorNum] * 1440) - currentEnc) / (newTime - lastTimes[motorNum]);
	}

	public void pause(long time) {
		try {
			wait(time);
 		} catch(Exception e) {

		}
	}

	public double[] getLastTimes() {
		return lastTimes;
	}

	public void setLastTimes(double[] lastTimes) {
		this.lastTimes = lastTimes;
	}

	public int[] getMeasuredVals() {
		return measuredVals;
	}

	public void setMeasuredVals(int[] measuredVals) {
		this.measuredVals = measuredVals;
	}

	public DcMotor[] getMotors() {
		return motors;
	}

	public void setMotors(DcMotor[] motors) {
		this.motors = motors;
	}

	public boolean isRunning() {
		return isRunning;
	}

	public void setIsRunning(boolean isRunning) {
		this.isRunning = isRunning;
	}

	public int[] getOverlaps() {
		return overlaps;
	}

	public void setOverlaps(int[] overlaps) {
		this.overlaps = overlaps;
	}
}
