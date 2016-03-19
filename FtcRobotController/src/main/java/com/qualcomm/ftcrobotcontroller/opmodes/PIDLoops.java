package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Matthew Raneri on 11/11/2015.
 */
public class PIDLoops extends Thread {
	//private long delayTime;
	private boolean isRunning;
	private double[] prevErrors;
	private double[] errors;
	private double[] results;
	private double[] measuredVals;
	private double[] setpoints;
	private double[] totalErrors;
	private double[] tuneP;
	private double[] tuneI;
	private double[] tuneD;

	public PIDLoops(long delayTime) {
		//this.delayTime = delayTimeParameter;
		this.isRunning = true;
		this.errors = new double[4];
		this.errors = initializeArray(this.errors);
		this.results = new double[4];
		this.results = initializeArray(this.results);
		this.measuredVals = new double[4];
		this.measuredVals = initializeArray(this.measuredVals);
		this.totalErrors = new double[4];
		this.totalErrors = initializeArray(this.totalErrors);
		this.setpoints = new double[4];
		this.setpoints = initializeArray(this.setpoints);
		this.prevErrors = new double[4];
		this.prevErrors = initializeArray(this.prevErrors);
		this.tuneP = new double[4];
		this.tuneP = initializeArray(this.tuneP);
		this.tuneI = new double[4];
		this.tuneI = initializeArray(this.tuneI);
		this.tuneD = new double[4];
		this.tuneD = initializeArray(this.tuneD);
	}

	public double[] initializeArray(double[] array) {
		for (int i = 0; i < array.length; i++) {
			array[i] = 0;
		}
		return array;
	}
	/*
	previous_error = 0
	integral = 0
	start:
  		error = setpoint - measured_value
  		integral = integral + error*dt
  		derivative = (error - previous_error)/dt
  		output = Kp*error + Ki*integral + Kd*derivative
  		previous_error = error
  		wait(dt)
  	goto start
	 */

	@Override
	public void run() {
		while (isRunning) {
			for (int i = 0; i < errors.length; i++) {
				errors[i] = setpoints[i] - measuredVals[i];
				errors[i] = (Math.round(errors[i] * 1000) / 1000);
				//if((errors[i] < .1 && errors[i] > 0)  && ((Math.abs(errors[i]))>=0.002) && (errors[i] > -.1 && errors[i] < 0)) errors[i] = 0.1;
				//else if((errors[i] < .002 && errors[i] > 0)) errors[i]=0;
				totalErrors[i] += errors[i];
				results[i] = (tuneP[i] * errors[i]) + (tuneI[i] * totalErrors[i]) + (tuneD[i] * ((errors[i] - prevErrors[i])));
				prevErrors[i] = errors[i];
			}
		}
	}

	public boolean isRunning() {
		return isRunning;
	}

	public void setIsRunning(boolean isRunning) {
		this.isRunning = isRunning;
	}

	public double[] getPrevErrors() {
		return prevErrors;
	}

	public void setPrevErrors(double[] prevErrors) {
		this.prevErrors = prevErrors;
	}

	public double[] getErrors() {
		return errors;
	}

	public void setErrors(double[] errors) {
		this.errors = errors;
	}

	public double[] getResults() {
		return results;
	}

	public void setResults(double[] results) {
		this.results = results;
	}

	public double[] getMeasuredVals() {
		return measuredVals;
	}

	public void setMeasuredVals(double[] measuredVals) {
		this.measuredVals = measuredVals;
	}

	public double[] getTotalErrors() {
		return totalErrors;
	}

	public void setTotalErrors(double[] totalErrors) {
		this.totalErrors = totalErrors;
	}

	public double[] getTuneP() {
		return tuneP;
	}

	public void setTuneP(double[] tuneP) {
		this.tuneP = tuneP;
	}

	public double[] getTuneI() {
		return tuneI;
	}

	public void setTuneI(double[] tuneI) {
		this.tuneI = tuneI;
	}

	public double[] getTuneD() {
		return tuneD;
	}

	public void setTuneD(double[] tuneD) {
		this.tuneD = tuneD;
	}

	public double[] getSetpoints() {
		return setpoints;
	}

	public void setSetpoints(double[] setpoints) {
		this.setpoints = setpoints;
	}
}
