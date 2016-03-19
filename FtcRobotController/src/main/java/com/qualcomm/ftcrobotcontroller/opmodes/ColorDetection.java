package com.qualcomm.ftcrobotcontroller.opmodes;

import android.provider.Settings;

import com.qualcomm.ftcrobotcontroller.GlobalVariables;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;

/**
 * Created by Matthew on 11/21/2015.
 */
public class ColorDetection {

	public final static boolean COLOR_RED = true;
	public final static boolean COLOR_BLUE = false;

	private MatOfPoint con;
	private Mat filteredImg;
	private ArrayList<MatOfPoint> contours;
	private boolean color;

	private int centerX;
	private int centerY;

	private double LowH;
	private double HighH;

	private double LowS;
	private double HighS;

	private double LowV;
	private double HighV;

	public ColorDetection(boolean color) {
		centerX = 0;
		centerY = 0;
		this.color = color;
		if (color == COLOR_RED) {
			LowH = 9;
			HighH = 170;
			LowS = 126;
			HighS = 140.385;
			LowV = 244;
			HighV = 255;
		} else {
			LowH = 82;
			HighH = 115;
			LowS = 200;
			HighS = 255;
			LowV = 200;
			HighV = 255;
		}
	}
	public boolean caploop() {
		if(GlobalVariables.CURRENT_FRAME == null) {
			return false;
		}
		Mat img = GlobalVariables.CURRENT_FRAME;
		Mat imgHSV = new Mat();
		Imgproc.blur(img, imgHSV, new Size(7, 7));
		Imgproc.cvtColor(img, imgHSV, Imgproc.COLOR_RGB2HSV);
		Scalar minVals = new Scalar(LowH, LowS, LowV);
		Scalar maxVals = new Scalar(HighH, HighS, HighV);

		if(color == COLOR_RED) {
			Mat mask1 = new Mat();
			Mat mask2 = new Mat();
			Scalar tempMinVals = new Scalar(0, LowS, LowV);
			Scalar tempMaxVals = new Scalar(LowH, HighS, HighV);
			Core.inRange(imgHSV, tempMinVals, tempMaxVals, mask1);
			tempMinVals = new Scalar(HighH, LowS, LowV);
			tempMaxVals = new Scalar(180, HighS, HighV);
			Core.inRange(imgHSV, tempMinVals, tempMaxVals, mask2);

			Imgproc.erode(mask1, mask1, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
			Imgproc.dilate(mask1, mask1, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));

			Imgproc.erode(mask2, mask2, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
			Imgproc.dilate(mask2, mask2, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));

			GlobalVariables.CURRENT_MASK = mask2;
		} else {

			Mat mask = new Mat();
			Core.inRange(imgHSV, minVals, maxVals, mask);
			Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
			Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
			//filteredImg = mask;
			GlobalVariables.CURRENT_MASK = mask;

		}
		contours = new ArrayList<MatOfPoint>();

		Imgproc.findContours(GlobalVariables.CURRENT_MASK, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		if (contours.size() > 0) {
			for (MatOfPoint contour : contours) {
				con = contour;
				if (Imgproc.contourArea(contour) > Imgproc.contourArea(con)) {
					con = contour;
				}
			}
		} else {
			con = null;
		}

		return true;
	}

	public Point getCenter() {
		if(con == null) {
			return new Point(1337, 1337);
		}
		Moments m = Imgproc.moments(con);
		return new Point(m.m10 / m.m00, m.m01 / m.m00);
	}



	public double getLowH() {
		return LowH;
	}

	public void setLowH(double lowH) {
		LowH = lowH;
	}

	public double getHighH() {
		return HighH;
	}

	public void setHighH(double highH) {
		HighH = highH;
	}

	public double getLowS() {
		return LowS;
	}

	public void setLowS(double lowS) {
		LowS = lowS;
	}

	public double getHighS() {
		return HighS;
	}

	public void setHighS(double highS) {
		HighS = highS;
	}

	public double getLowV() {
		return LowV;
	}

	public void setLowV(double lowV) {
		LowV = lowV;
	}

	public double getHighV() {
		return HighV;
	}

	public void setHighV(double highV) {
		HighV = highV;
	}

	public MatOfPoint getCon() {
		return con;
	}

	public void setCon(MatOfPoint con) {
		this.con = con;
	}
}
