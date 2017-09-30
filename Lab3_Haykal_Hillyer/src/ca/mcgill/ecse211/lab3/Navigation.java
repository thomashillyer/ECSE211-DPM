package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 75;
	private static final int ACCELERATION = 200;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double currX = 0.0;
	private double currY = 0.0;
	private double currTheta = 0.0;

	private boolean isNavigating = false;

	private static final double WHEEL_BASE = 11.5;
	private static final double WHEEL_RADIUS = 2.1;
	private static final double TILE_LENGTH = 30.48;

	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

	}

	public void travelTo(double x, double y) {
		// This method causes the robot to travel to the absolute field location (x, y),
		// specified in tile points.This method should continuously call turnTo(double
		// theta) and then set the motor speed to forward(straight). This will make sure
		// that your heading is updated until you reach your exact goal. This method
		// will poll the odometer for information
		isNavigating = true;

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

		isNavigating = false;
	}

	public void turnTo(double theta) {
		// this method should turn (on point) to the absolute heading theta, by using a
		// minimal angle

		isNavigating = true;

		if (theta < -Math.PI) {
			theta += Math.PI * 2;
		} else if (theta > Math.PI) {
			theta -= Math.PI * 2;
		}

		theta = theta * 180 / Math.PI;

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// turn to the left if angle is negative
		if (theta < 0) {
			rightMotor.rotate(convertAngle(WHEEL_RADIUS, WHEEL_BASE, -theta), true);
			leftMotor.rotate(-convertAngle(WHEEL_RADIUS, WHEEL_BASE, -theta), false);
		}
		// turn to the right if angle is positive
		else {
			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, WHEEL_BASE, theta), true);
			leftMotor.rotate(convertAngle(WHEEL_RADIUS, WHEEL_BASE, theta), false);
		}
		
		isNavigating = false;
	}

	public boolean isNavigating() {
		// method returns true if another thread has called travelTo() or turnTo() and
		// the method has yet to return; false otherwise

		return isNavigating;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	public void run() {
		odometer.setX(0);
		odometer.setY(0);
		odometer.setTheta(0);

		leftMotor.stop(true);
		leftMotor.setAcceleration(ACCELERATION);
		rightMotor.stop(true);
		rightMotor.setAcceleration(ACCELERATION);

		// rename one of these just "map" to test
		int[][] map1 = { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
		int[][] map2 = { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
		int[][] map = { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
		int[][] map4 = { { 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } };

		travelTo(map[0][0], map[0][1]);
		travelTo(map[1][0], map[1][1]);
		travelTo(map[2][0], map[2][1]);
		travelTo(map[3][0], map[3][1]);
		travelTo(map[4][0], map[4][1]);

		// testing
		// travelTo(0, 1);
		// travelTo(1, 1);
	}
}
