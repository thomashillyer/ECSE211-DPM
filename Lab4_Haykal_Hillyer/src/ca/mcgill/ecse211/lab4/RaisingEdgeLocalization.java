package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class RaisingEdgeLocalization extends Thread implements UltrasonicController {

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	private static final double RADIUS = 2.1;
	private static final double TRACK = 11.9;

	private static final int ROTATESPEED = 50;
	private static final int ACCELERATION = 50;
	private static final int horizontalConstant = 35;
	private static final int noiseMargin = 2;

	private static final int FILTER_OUT = 20;
	private int filterControl;
	private int distanceUS = 255;

	private boolean detectRaisingEdge = false;

	private double fallingTheta;
	private double risingTheta;
	private double deltaTheta;

	public RaisingEdgeLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}

	public void run() {
		// Set the acceleration of both motors
		leftMotor.stop();
		leftMotor.setAcceleration(ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(ACCELERATION);

		// Set the speed of each motor
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);

		// Rotate the robot 360 degrees, and don't block the thread while
		// turning
		rightMotor.rotate(-convertAngle(RADIUS, TRACK, 360), true);
		leftMotor.rotate(convertAngle(RADIUS, TRACK, 360), true);
		
		//detect falling and rising edge, calculate the angle by which the robot should turn to face north
		detectFallingAndRisingEdge();
		
		//turn the robot by the angle by which it will make it head north
		rightMotor.rotate(convertAngle(RADIUS, TRACK, deltaTheta * 180.0 / Math.PI), true);
		leftMotor.rotate(-convertAngle(RADIUS, TRACK, deltaTheta * 180.0 / Math.PI), false);	
	}
	
	/**
	 * This method detects a rising edge, when the sensor gets away from the wall, and a falling edge, when
	 * the sensor gets closer to the wall. In order to do so, if the distance returned by the robot is bigger
	 * than a fixed distance, which is the horizontal constant, and we still didn't detect a rising edge, this
	 * means that the robot heading is moving away from a wall. The angle returned by the odometer is saved. Then,
	 * if the robot has already detected a rising edge, and the distance measured by the robot gets lower than the
	 * constant, this means that the robot heading is getting closer to a wall, and a falling edge is detected. 
	 * The angle return by the odometer is then saved. Then, the method uses both angles to determine by how much
	 * the robot should turn to make it face north (0 degrees)
	 */
	private void detectFallingAndRisingEdge() {
		
		// As long as the robot is moving, don't leave this loop
		while (leftMotor.isMoving() && rightMotor.isMoving()) {

			if ((this.distanceUS >= horizontalConstant - noiseMargin) && detectRaisingEdge == false) {
				detectRaisingEdge = true;
				risingTheta = odometer.getTheta();
				Sound.beep();
				System.out.println("falling theta: " + risingTheta * 180.0 / Math.PI);
			} else if (this.distanceUS < horizontalConstant + noiseMargin && detectRaisingEdge == true) {
				detectRaisingEdge = false;
				fallingTheta = odometer.getTheta();
				Sound.beep();
				System.out.println("rising theta: " + fallingTheta * 180.0 / Math.PI);
			}
		}

		if (fallingTheta < risingTheta) {
			deltaTheta = (5.0 * Math.PI / 4.0) - (fallingTheta + risingTheta) / 2.0;
			//System.out.println("falling theta: " + fallingTheta);
			//System.out.println("rising theta: " + risingTheta);
			//System.out.println("delta theta1: " + deltaTheta);
		} else {
			deltaTheta = (Math.PI / 4.0) - (fallingTheta + risingTheta) / 2.0;
			//System.out.println("delta theta2: " + deltaTheta);
		}
		
		System.out.println("delta theta: " + deltaTheta * 180.0 / Math.PI);
		System.out.println("theta: " + odometer.getTheta() * 180.0 / Math.PI);
	}

	@Override
	public void processUSData(int distance) {
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: set the distance to the max
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

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}
}
