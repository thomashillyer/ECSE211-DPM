package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Odometer;
import ca.mcgill.ecse211.lab4.UltrasonicController;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class FallingEdgeLocalization extends Thread implements UltrasonicController {

	private static final int ROTATIONSPEED = 50;
	private static final int ACCELERATION = 50;
	private static final double RADIUS = 2.1;
	private static final double TRACK = 16;

	private final int horizontalConstant = 40;

	private static double alphaAngle = 0.0;
	private static double betaAngle = 0.0;
	private static double deltaTheta = 0.0;

	private int distanceUS = 255;
	private int filterControl;
	private static final int FILTER_OUT = 10;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	private boolean fallingEdgeDetected = false;

	public FallingEdgeLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;

	}

	/**
	 * The falling and rising edges are detected in this method. The rising edge
	 * is when the sensor goes from facing the wall to away from the wall and
	 * the falling edge is from away from the wall to towards the wall. If no
	 * falling edge is detected and the distance drops below the constant then a
	 * falling edge is said to have been detected. Then if a falling edge is
	 * detected and the distance rises above our horizontal constant, a rising
	 * edge is said to be detected. The angle return by the odometer is then
	 * saved. Then, the method uses both angles to determine by how much the
	 * robot should turn to make it face north (0 degrees)
	 */
	public void run() {

		leftMotor.stop();
		leftMotor.setAcceleration(ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(ACCELERATION);

		leftMotor.setSpeed(ROTATIONSPEED);
		rightMotor.setSpeed(ROTATIONSPEED);

		// rotate 360 degrees to scan walls
		leftMotor.rotate(convertAngle(RADIUS, TRACK, 360), true);
		rightMotor.rotate(-convertAngle(RADIUS, TRACK, 360), true);

		/*
		 *  while robot is rotating, compare the distance returned by the ultrasonic sensor to the 
		 *  threshold, and if it is less than threshold, a falling edge is detected. Save the angle and set
		 *  the flag to true, so when the distance gets bigger than the threshold, the program enters the second
		 *  if statement and save the anlge of the rising edge.
		 */
		while ((rightMotor.isMoving() && leftMotor.isMoving())) {

			if (!fallingEdgeDetected && (distanceUS <= horizontalConstant)) {
				alphaAngle = odometer.getTheta();
				Sound.beep();
				fallingEdgeDetected = true;

			} else if (fallingEdgeDetected && (distanceUS > horizontalConstant)) {
				betaAngle = odometer.getTheta();
				Sound.beep();
				fallingEdgeDetected = false;
			}
		}

		// compute the odometer's angular offset
		if (alphaAngle <= betaAngle) {
			deltaTheta = 5.0 * Math.PI / 4.0 - (alphaAngle + betaAngle) / 2.0;
		} else if (alphaAngle > betaAngle) {
			deltaTheta = Math.PI / 4.0 - (alphaAngle + betaAngle) / 2.0;
		}

		odometer.setTheta(odometer.getTheta() + deltaTheta);
		// System.out.println(deltaTheta);
		// rotate to face 0 degrees
		leftMotor.rotate(-convertAngle(RADIUS, TRACK, deltaTheta * 180 / Math.PI), true);
		rightMotor.rotate(convertAngle(RADIUS, TRACK, deltaTheta * 180 / Math.PI), false);

	}

	@Override
	public int readUSDistance() {
		return this.distanceUS;
	}

	@Override
	/**
	 * Filter any extraneous distances returned by the sensor
	 * 
	 * @param distance
	 *            the distance recorded by the US sensor
	 */
	public void processUSData(int distance) {

		if (betaAngle != 0.0) {
			distance = 255;
		} else {

			if (distance >= 255 && filterControl < FILTER_OUT) {
				// bad value, do not set the distance var, however do increment
				// the
				// filter value
				filterControl++;
			} else if (distance >= 255) {
				// We have repeated large values, so there must actually be
				// nothing
				// there: leave the distance alone

				distance = 255;

				this.distanceUS = distance;
			} else {
				// distance went below 255: reset filter and leave
				// distance alone.
				filterControl = 0;
				this.distanceUS = distance;

			}
		}
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}

}
