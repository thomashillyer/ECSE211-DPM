package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class RaisingEdgeLocalization extends Thread implements UltrasonicController{
	
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	private static final double RADIUS = 2.1;
	private static final double TRACK = 11.9;
	private static final double TILE_LENGTH = 30.48;
	
	private static final int FORWARDSPEED = 150;
	private static final int ROTATESPEED = 50;
	private static final int DIFFERENCESPEED = 50;
	private static final int ACCELERATION = 50;
	private static final int horizontalConstant = 40;
	private static final int noiseMargin = 2;
	
	
	private static final int FILTER_OUT = 20;
	private int filterControl;
	private int distanceUS;
	private double lastTheta;
	
	private boolean detectRaisingEdge = false;
	
	private double fallingTheta;
	private double risingTheta;
	private double deltaTheta;
	
	public RaisingEdgeLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer){
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}
	
	public void run() {

		leftMotor.stop();
		leftMotor.setAcceleration(ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(ACCELERATION);
		
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);
		
		//turnTo(2 * Math.PI);
		leftMotor.rotate(convertAngle(RADIUS, TRACK, 360), true);
		rightMotor.rotate(-convertAngle(RADIUS, TRACK, 360), true);
		lastTheta = 0;
		while(leftMotor.isMoving() && rightMotor.isMoving()){
			double theta = odometer.getTheta() * 180.0 / Math.PI;
			
			System.out.println(theta + "," + this.distanceUS);
			
			lastTheta = theta;
			
			if((this.distanceUS >= horizontalConstant - noiseMargin) && detectRaisingEdge == false){
				detectRaisingEdge = true;
				risingTheta =  odometer.getTheta() * 180.0 / Math.PI;
				Sound.beep();
				System.out.println("falling theta: " + risingTheta);
			}else if(this.distanceUS < horizontalConstant + noiseMargin && detectRaisingEdge == true){
				detectRaisingEdge = false;
				fallingTheta =  odometer.getTheta() * 180.0 / Math.PI;
				Sound.beep();
				System.out.println("rising theta: " + fallingTheta);
			}

		}
		
		if(fallingTheta < risingTheta){
			deltaTheta = 225.0 - (fallingTheta + risingTheta) / 2.0;
			System.out.println("falling theta: " + fallingTheta);
			System.out.println("rising theta: " + risingTheta);
			System.out.println("delta theta1: " + deltaTheta);
		}else{
			deltaTheta = 45.0 - (fallingTheta + risingTheta) / 2.0;
			System.out.println("delta theta2: " + deltaTheta);
		}
		
		leftMotor.rotate(-convertAngle(RADIUS, TRACK, deltaTheta), true);
		rightMotor.rotate(convertAngle(RADIUS, TRACK, deltaTheta), false);	
		
	}
	
	@Override
	public void processUSData(int distance) {
		//if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		//} else if (distance >= 255) {
			if (distance >= 255){
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distanceUS = 255;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distanceUS = distance;
		}

	}

	@Override
	public int readUSDistance() {
		return this.distanceUS;
	}
	
	private void turnTo(double theta) {
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);

		if (theta > Math.PI) {
			theta -= 2 * Math.PI;

		} else if (theta <= -Math.PI) {
			theta += 2 * Math.PI;
		}

		// turn left is angle is negative, else turn right
		if (theta < 0) {
			leftMotor.rotate(-convertAngle(RADIUS, TRACK, -(theta * 180) / Math.PI), true);
			rightMotor.rotate(convertAngle(RADIUS, TRACK, -(theta * 180) / Math.PI), false);
		} else {
			leftMotor.rotate(convertAngle(RADIUS, TRACK, (theta * 180) / Math.PI), true);
			rightMotor.rotate(-convertAngle(RADIUS, TRACK, (theta * 180) / Math.PI), false);
		}
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}
}
