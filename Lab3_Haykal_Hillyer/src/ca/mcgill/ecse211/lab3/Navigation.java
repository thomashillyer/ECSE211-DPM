package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {
	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 50;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double currX = 0.0;
	private double currY = 0.0;
	private double currTheta = 0.0;

	private static final double WHEEL_BASE = 11.8;
	private static final double WHEEL_RADIUS = 2.1;
	private static final double TILE_LENGTH = 30.48;

	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

	}

	public void run() {
		odometer.setX(0);
		odometer.setY(0);
		odometer.setTheta(0);

		leftMotor.stop(true);
		rightMotor.stop(true);

		// rename one of these just "map" to test
		int[][] map1 = { { 2, 1 }, { 1, 1 }, { 1, 2 }, { 2, 0 } };

		travelTo(map1[0][0], map1[0][1]);
		travelTo(map1[1][0], map1[1][1]);
		travelTo(map1[2][0], map1[2][1]);
		travelTo(map1[3][0], map1[3][1]);

	}

	public void travelTo(double x, double y) {
		// This method causes the robot to travel to the absolute field location (x, y),
		// specified in tile points.This method should continuously call turnTo(double
		// theta) and then set the motor speed to forward(straight). This will make sure
		// that your heading is updated until you reach your exact goal. This method
		// will poll the odometer for information

		currX = odometer.getX();
		currY = odometer.getY();
		currTheta = odometer.getTheta();

		double deltaX = (x * TILE_LENGTH) - currX;
		double deltaY = (y * TILE_LENGTH) - currY;
		
		double deltaTheta = Math.atan2(deltaX, deltaY) - currTheta;

		turnTo(deltaTheta);

		// find hypotenuse
		double distToTravel = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);
		distToTravel = Math.sqrt(distToTravel);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		rightMotor.rotate(convertDistance(WHEEL_RADIUS, distToTravel), true);
		leftMotor.rotate(convertDistance(WHEEL_RADIUS, distToTravel), false);

		leftMotor.stop(true);
		rightMotor.stop(true);

	}

	public void turnTo(double theta) {
		// this method should turn (on point) to the absolute heading theta, by using a
		// minimal angle
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		if (theta <= -Math.PI) {
			theta += Math.PI * 2;
		} else if (theta > Math.PI) {
			theta -= Math.PI * 2;
		}

		theta = theta * 180.0 / Math.PI;
		
		// turn to the left if angle is negative
		if (theta < 0) {
			leftMotor.rotate(-convertAngle(WHEEL_RADIUS, WHEEL_BASE, -theta), true);
			rightMotor.rotate(convertAngle(WHEEL_RADIUS, WHEEL_BASE, -theta), false);
		}
		// turn to the right if angle is positive
		else {
			leftMotor.rotate(convertAngle(WHEEL_RADIUS, WHEEL_BASE, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, WHEEL_BASE, theta), false);
		}

	}


	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
