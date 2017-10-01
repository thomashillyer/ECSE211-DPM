package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigationObstacle extends Thread implements UltrasonicController {

	private static Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;

	private final double RADIUS = 2.1;
	private final double TRACK = 11.5;
	private static double Tile_Length = 30.48;

	private static final int FWSPEED = 250;
	private static final int DTSPEED = 50;
	private static final int RTSPEED = 150;
	private static final int MTRACCEL = 200;

	private static boolean isNavigating = true;

	private static final int FILTER_OUT = 20;

	private static final double OdometerThreshold = 0.2;

	private int distance;
	private int filterControl;
	private int backwardControl;

	private final int bandCenter;
	private final int bandwidth;
	private final double anglePosition;

	public NavigationObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.bandwidth = 3;
		this.bandCenter = 21;
		this.backwardControl = 5;
		this.anglePosition = 1.5;

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

	@Override
	public void processUSData(int distance) {

	}

	public void run() {

		leftMotor.stop();
		leftMotor.setAcceleration(MTRACCEL);

		rightMotor.stop();
		rightMotor.setAcceleration(MTRACCEL);

		travelTo(1 * Tile_Length, 1 * Tile_Length);
		travelTo(0 * Tile_Length, 2 * Tile_Length);
		travelTo(2 * Tile_Length, 2 * Tile_Length);
		travelTo(2 * Tile_Length, 1 * Tile_Length);
		travelTo(1 * Tile_Length, 0 * Tile_Length);

	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}

	private void travelTo(double x, double y) {
		isNavigating = true;
		double deltaX = x - odometer.getX();
		double deltaY = y - odometer.getY();

		double minimumAngle = Math.atan2(deltaX, deltaY) - odometer.getTheta();

		turnTo(minimumAngle);

		// calculate the distance to next point
		double distance = Math.hypot(deltaX, deltaY);

		// move to the next point
		leftMotor.setSpeed(FWSPEED);
		rightMotor.setSpeed(FWSPEED);
		leftMotor.rotate(convertDistance(RADIUS, distance), true);
		rightMotor.rotate(convertDistance(RADIUS, distance), true);

		while ((((odometer.getX() >= x + OdometerThreshold) || (odometer.getX() < x - OdometerThreshold))
				&& ((odometer.getY() >= y + OdometerThreshold) || (odometer.getY() < y + OdometerThreshold)))) {

		}

		leftMotor.stop(true);
		rightMotor.stop(true);
		isNavigating = false;
	}

	private void turnTo(double theta) {
		leftMotor.setSpeed(RTSPEED);
		rightMotor.setSpeed(RTSPEED);

		if (theta > Math.PI) {
			theta -= 2 * Math.PI;

		} else if (theta <= -Math.PI) {
			theta += 2 * Math.PI;
		}

		if (theta < 0) {
			leftMotor.rotate(-convertAngle(RADIUS, TRACK, -(theta * 180) / Math.PI), true);
			rightMotor.rotate(convertAngle(RADIUS, TRACK, -(theta * 180) / Math.PI), false);
		} else {
			leftMotor.rotate(convertAngle(RADIUS, TRACK, (theta * 180) / Math.PI), true);
			rightMotor.rotate(-convertAngle(RADIUS, TRACK, (theta * 180) / Math.PI), false);
		}
	}

}
