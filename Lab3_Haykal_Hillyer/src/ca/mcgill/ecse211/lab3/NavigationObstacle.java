package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigationObstacle extends Thread implements UltrasonicController {
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 75;
	private static final int ACCELERATION = 200;
	private static final int FILTER_OUT = 20;
	private static final int BAND_CENTER = 10;
	private static final int BAND_WIDTH = 5;
	private static final int MAX_DISTANCE = 150; // max distance to allow through the filter

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double currX = 0.0;
	private double currY = 0.0;
	private double currTheta = 0.0;

	private static int filterControl;
	private int distance;

	private boolean isNavigating = false;

	private static final double WHEEL_BASE = 11.5;
	private static final double WHEEL_RADIUS = 2.1;
	private static final double TILE_LENGTH = 30.48;

	public NavigationObstacle(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		this.filterControl = 0;

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
		// difference is set this to true so it doesnt block
		leftMotor.rotate(convertDistance(WHEEL_RADIUS, distToTravel), true);

		// isMoving is set true while the motor is trying to rotate, from the rotate
		// method above
		while (leftMotor.isMoving() && rightMotor.isMoving())
			;

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
			leftMotor.rotate(-convertAngle(WHEEL_RADIUS, WHEEL_BASE, -theta), true);
			rightMotor.rotate(convertAngle(WHEEL_RADIUS, WHEEL_BASE, -theta), false);
		}
		// turn to the right if angle is positive
		else {
			leftMotor.rotate(convertAngle(WHEEL_RADIUS, WHEEL_BASE, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, WHEEL_BASE, theta), false);
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

	@Override
	public void processUSData(int distance) {

		filterData(distance);
		
		System.out.println(this.distance);

		if(this.distance < BAND_CENTER - BAND_WIDTH){
			leftMotor.stop();
			rightMotor.stop();
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

	private void filterData(int distance) {

		if (distance >= MAX_DISTANCE && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= MAX_DISTANCE) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;

			float floatDistance = distance / (float) 1.4;

			this.distance = (int) floatDistance;
		}
	}
}
