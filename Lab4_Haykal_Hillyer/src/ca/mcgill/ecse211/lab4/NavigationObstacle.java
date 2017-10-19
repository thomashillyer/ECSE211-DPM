package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigationObstacle extends Thread implements UltrasonicController {

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor, sensorMotor;

	private static final double RADIUS = 2.1;
	private static final double TRACK = 12.3;
	private static final double TILE_LENGTH = 30.48;

	private static final int FORWARDSPEED = 150;
	private static final int ROTATESPEED = 50;
	private static final int DIFFERENCESPEED = 50;
	private static final int ACCELERATION = 50;

	private static boolean wallFollowingMode = false;
	
	//A constant which controls the "going backwards" of the robot in the bang bang controller
	private final int backwardControl = 5;

	private static final int FILTER_OUT = 20;
	private final int bandCenter = 13;

	private float distanceError;
	private int distanceUS;
	private int filterControl;
	private double savedAngle = 0;
	private int wayPointsCounter = 0;

	private int[][] wayPoints = { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };

	public NavigationObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3LargeRegulatedMotor sensorMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
	}

	/**
	 * @return This method returns the distance measured by the sensor
	 */
	@Override
	public int readUSDistance() {
		return this.distanceUS;
	}

	/**
	 * This method takes as input a distance (integer), and filters the distance
	 * by ignoring distances that are larger than 255 for a given number of
	 * time. Therefore, this method updates the distance of the robot every
	 * time, except when a distance of 255 or more is measured less than 20
	 * times.
	 * 
	 * @param interger
	 *            distance of the robot from the wall
	 */

	@Override
	public void processUSData(int distance) {
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distanceUS = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distanceUS = distance;
		}

		// calculate the error of the robot by subtracting the actual distance
		// from the band center
		this.distanceError = bandCenter - this.distanceUS;
	}

	// This method runs the thread. It starts by stopping all motors, and then
	// by calling the travelTo method for
	// each given way point.
	public void run() {

		leftMotor.stop();

		rightMotor.stop();

		sensorMotor.stop();
		sensorMotor.setAcceleration(ACCELERATION);

		for (wayPointsCounter = 0; wayPointsCounter < wayPoints.length; wayPointsCounter++) {
			travelTo(wayPoints[wayPointsCounter][0] * TILE_LENGTH, wayPoints[wayPointsCounter][1] * TILE_LENGTH);
		}
	}

	/**
	 * This method causes the robot to travel to the absolute field location (x,
	 * y), specified in tile points. It calls turnTo(double theta) in order to
	 * rotate the robot so it would be heading its destination. Then, the robot
	 * moves to the next point. While traveling, the method checks for any
	 * obstacle using the data from the sensor. If an object is detected, the
	 * robot rotates the sensor towards the object, and starts the wall
	 * following technique. Once the robot gets through the object, it continues
	 * traveling to the way point.
	 * 
	 * @param x
	 *            The new x destination of the robot
	 * @param y
	 *            The new y destination of the robot
	 */

	private void travelTo(double x, double y) {
		// calculating the information needed (destination - current) for both y
		// and x, in order to calculate the minimum angle using arctan
		double deltaX = x - odometer.getX();
		double deltaY = y - odometer.getY();

		// calculating the minimum angle using Math.atan2 method
		double minimumAngle = Math.atan2(deltaX, deltaY) - odometer.getTheta();

		// rotate the robot towards its new way point
		turnTo(minimumAngle);

		// calculate the distance to next point using the built in pythagore
		// theorem
		double distance = Math.hypot(deltaX, deltaY);

		// travel to the next point, and don't wait until the action is
		// complete. So the boolean in both rotate method should be true
		leftMotor.setSpeed(FORWARDSPEED);
		rightMotor.setSpeed(FORWARDSPEED);
		leftMotor.rotate(convertDistance(RADIUS, distance), true);
		rightMotor.rotate(convertDistance(RADIUS, distance), true);

		// since rotate doesn't wait, we can check if there is an obstacle on
		// the way to the way point
		// while the motor is still traveling to the way point don't exit the
		// while loop
		while ((rightMotor.isMoving() && leftMotor.isMoving())) {

			/*
			 * This condition is satisfied when we are not in a wallfollowing
			 * mode, and the robot detects an object. It uses the distanceUS
			 * which is distance returned by the sensor and compared it to the
			 * band center. If the distance is less than the band center, save
			 * the current angle of robot (rotation), set the mode to
			 * wallfollowing, stops the motors, and then let the robot rotate 90
			 * degrees, and the sensor -90 degrees (pointing towards the object)
			 */
			if ((distanceUS < bandCenter) && (!wallFollowingMode)) {

				savedAngle = odometer.getTheta();

				wallFollowingMode = true;
				leftMotor.stop(true);
				rightMotor.stop(true);
				leftMotor.rotate(convertAngle(RADIUS, TRACK, 90), true);
				rightMotor.rotate(-convertAngle(RADIUS, TRACK, 90), false);
				sensorMotor.rotate(-90);

			}

			/*
			 * This condition is satisfied when the mode is set to wall
			 * following. What's inside this if statement is a bang-bang
			 * controller. The error being positive means that the robot
			 * is close to the wall. Now to check if the robot is extremely
			 * close to the wall, we check if the error is greater than half the
			 * band center + a constant. This constant gives the flexibility to actually
			 * tweek the variable when testing. Therefore, if the error is positive and extremely
			 * close to the wall, go backwards, else if the error is positive and fairly far from
			 * the wall, turn left, otherwise turn right.
			 * 
			 */
			if (wallFollowingMode) {
				if (distanceError >= 0 && distanceError < bandCenter / 2 + backwardControl) {

					rightMotor.setSpeed(FORWARDSPEED - DIFFERENCESPEED);
					leftMotor.setSpeed(FORWARDSPEED + DIFFERENCESPEED);
					rightMotor.forward();
					leftMotor.forward();
				}

				else if (distanceError >= bandCenter / 2 - backwardControl) {
					rightMotor.setSpeed(FORWARDSPEED + DIFFERENCESPEED);
					leftMotor.setSpeed(FORWARDSPEED + DIFFERENCESPEED);
					rightMotor.backward();
					leftMotor.backward();
				}

				else if (distanceError < 0) {
					leftMotor.setSpeed(FORWARDSPEED - DIFFERENCESPEED);
					rightMotor.setSpeed(FORWARDSPEED + DIFFERENCESPEED);
					rightMotor.forward();
					leftMotor.forward();
				}
			}

			/*
			 * this condition is satisfied when the mode is set to wall
			 * following, and when the difference between the saved angle and
			 * actual angle is greater then 45 degree. In other words, when the
			 * robot traversed the wall, finished traversing the object and
			 * needs to continue traveling to the waypoint. for example, if the
			 * saved angle is 0 degrees, the robot turns at first 90 degrees so
			 * difference is 90. Then the robot starts rotating around the
			 * object until it goes back to an angle of 90 with a differnce of
			 * 0. It will continue rotating around the object until the reported
			 * angle is greater than 135 degrees. Then it will continue to
			 * travel to the way point.
			 */
			if (wallFollowingMode && (savedAngle - odometer.getTheta() > Math.PI / 4)) {
				leftMotor.stop();
				rightMotor.stop();
				sensorMotor.rotate(90, true);

				wallFollowingMode = false;

				// fixes the counter, since while wall following mode is on, the
				// robot hasn't reached another waypoint.
				wayPointsCounter--;
				break;
			}
		}

		leftMotor.stop(true);
		rightMotor.stop(true);
	}

	/**
	 * This method takes an angle theta as input. It starts by setting the speed
	 * of both motors. The angle is between -360 and 360 degrees, but since this
	 * methods should always make the robot turn with the minimum angle
	 * possible, theta should be updated. If theta is greater than 180 degrees,
	 * instead of turning positively, the robot turns negatively by an angle of
	 * theta - 360. And if theta is less than or equal to -180, instead of
	 * turning negatively, the robot turns positevly by an angle of theta + 360.
	 * 
	 * @param theta
	 *            The angle by which the cart should turn.
	 */
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
