package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.GlobalVariables;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/**
 * Created by Matthew on 2/1/2016.
 */
public class TrackingThread extends Thread {

	private ColorDetection cd;
	private boolean detected;
	private int xPos;
	private int yPos;
	private ArrayList<Point> points = new ArrayList();
	private Point center;


	public TrackingThread() {
		cd = new ColorDetection(ColorDetection.COLOR_BLUE);
		detected = false;
	}

	@Override
	public void run() {
		while(GlobalVariables.isStillRunning) {
			cd.caploop();
			if (cd.getCon() != null && Imgproc.contourArea(cd.getCon()) > 100) {
				detected = true;
				Point center;
				//I want to store the centroid of the mask in the point 'center'
				//I also have access to the contour I want to use above
				//but however I try to compute the moments, I get a crash
				Rect rect = Imgproc.boundingRect(cd.getCon());
				center = new Point((rect.x + rect.width) / 2, (rect.y + rect.height) / 2);
				points.add(center);
				if(points.size() > 3) {
					points.remove(1);
				}

				Point average = new Point();
				for(Point p : points) {
					average.x = average.x + p.x;
					average.y = average.y + p.y;
				}

				yPos = (int)(average.y / 3);
				xPos = (int)(average.x / 3);
			} else {
				points.clear();
				detected = false;
			}
		}
	}

	public MatOfPoint getCon() {
		return cd.getCon();
	}

	public boolean isDetected() {
		return detected;
	}

	public void setDetected(boolean detected) {
		this.detected = detected;
	}

	public int getxPos() {
		return xPos;
	}

	public void setxPos(int xPos) {
		this.xPos = xPos;
	}

	public int getyPos() {
		return yPos;
	}

	public void setyPos(int yPos) {
		this.yPos = yPos;
	}
}
//hrhrjykkfydy