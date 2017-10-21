package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightSensorLocalization extends Thread {

	private static final Port lightSampler = LocalEV3.get().getPort("S2");

	private SensorModes colosSamplerSensor = new EV3ColorSensor(lightSampler);
	private SampleProvider colorSensorValue = colosSamplerSensor.getMode("Red");

	private float[] colorSensorData = new float[colosSamplerSensor.sampleSize()];

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	private static final double RADIUS = 2.1;
	private static final double TRACK = 11.45;

	private static final int FORWARDSPEED = 100;
	private static final int ROTATESPEED = 50;
	private static final int ACCELERATION = 200;
	private static final double CENTERDISTANCE = 14;

	// data
	private int filterCounter = 0;
	private float oldValue = 0;
	private int derivativeThreshold = -30;
	private int lineCounter = 0;
	private double xminus, xplus, yminus, yplus;
	private double thetax, thetay;
	private double x, y;
	private double deltaThetaY;

	private Navigation navigation;

	public LightSensorLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		navigation = new Navigation(odometer, leftMotor, rightMotor);
	}

	public void run() {
		Button.waitForAnyPress();

		// Set the acceleration of both motor
		leftMotor.stop();
		leftMotor.setAcceleration(ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(ACCELERATION);

		// Adjust the robot position, before it starts to rotate to ensure that
		// the light sensor will cross 4 black lines
		adjustRobotStartingPosition();

		// set the robot wheel's rotation speed to both motors
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);

		// rotate the robot 360 degrees
		leftMotor.rotate(convertAngle(RADIUS, TRACK, 360), true);
		rightMotor.rotate(-convertAngle(RADIUS, TRACK, 360), true);

		// while rotating, get the angles which are used to correct the robot's
		// position and orientation
		determineLocalizationAngles();

		// turn and travel to (0, 0) which is the first intersection
		travelToDestination(0, 0);

		// wait until the robot reached (0 ,0)
		while ((rightMotor.isMoving() && leftMotor.isMoving()))
			;

		// let the robot head north
		navigation.turnTo(-odometer.getTheta());

	}

	/**
	 * This method calculates the robot's actual x and y position using the
	 * angle values determine earlier using trigonometry, and then determine the
	 * theta by which the odometer is off. Then, it updates the x, y and theta
	 * values of the odometer in order to fix them. Then, it uses the navigation
	 * travelTo method to travel to the inputted destination.
	 * 
	 * @param xdestination
	 * @param ydestination
	 */
	private void travelToDestination(int xdestination, int ydestination) {

		thetay = yminus - yplus;
		thetax = xplus - xminus;

		x = -CENTERDISTANCE * Math.cos(thetay / 2.0);
		y = -CENTERDISTANCE * Math.cos(thetax / 2.0);
		deltaThetaY = (Math.PI / 2.0) - yminus + Math.PI + (thetay / 2.0);

		odometer.setX(x);
		odometer.setY(y);
		odometer.setTheta(odometer.getTheta() + deltaThetaY);

		navigation.travelTo(xdestination, ydestination);
	}

	/**
	 * while the robot is rotating on itself, get the measured values of the
	 * light sensor. A black line is detected when the derivative less than a
	 * threshold. When it is detected, a filter is added because sometimes,
	 * since the frequency of the value returned by the robot is high, the
	 * derivative can be less than the threshold for 2 or 3 consecutive returned
	 * values. This filter makes sure that we enter the if statement only the
	 * first time the derivative is less than threshold. Moreover, a line
	 * counter is added to determine which theta is return by the odometer
	 * (theta (x-), theta (y+), theta (x+), and theta (y-) respectively).
	 */
	private void determineLocalizationAngles() {

		while (leftMotor.isMoving() && rightMotor.isMoving()) {
			// fetching the values from the color sensor
			colorSensorValue.fetchSample(colorSensorData, 0);

			// getting the value returned from the sensor, and multiply it by
			// 1000 to scale
			float value = colorSensorData[0] * 1000;

			// computing the derivative at each point
			float diff = value - oldValue;

			// storing the current value, to be able to get the derivative on
			// the next iteration
			oldValue = value;

			if (diff < derivativeThreshold && filterCounter == 0) {
				Sound.beep();
				filterCounter++;
				lineCounter++;

				if (lineCounter == 1) {
					xminus = odometer.getTheta();

				} else if (lineCounter == 2) {
					yplus = odometer.getTheta();

				} else if (lineCounter == 3) {
					xplus = odometer.getTheta();

				} else if (lineCounter == 4) {
					yminus = odometer.getTheta();

				}
			} else if (diff < derivativeThreshold && filterCounter > 0) {
				filterCounter++;
			} else if (diff > derivativeThreshold) {
				filterCounter = 0;
			}

		}
	}

	/**
	 * This method adjusts the robot position before it starts rotating. It
	 * ensures that the light sensor, mounted on the back of the robot, runs
	 * over 4 black lines. It order to so, the robot, facing north after the
	 * ultrasonic localization, moves forward until it detects a line, and then
	 * move backwards 1.5 times its center distance. This, ensures that the
	 * robot is close enough to the horizontal black line facing the robot.
	 * Then, the method makes the robot turn 90 degrees and repeat the same
	 * procedure, so it can be placed close enough to the vertical line. Now the
	 * robot it close enough to the first intersection.
	 */
	private void adjustRobotStartingPosition() {

		// This method gets the data from the light sensor when the robot is
		// moving forward, and returns when a black line is detected
		detectBlackLine();

		// Move the robot backwards 1.5 * its center distance
		rightMotor.rotate(-convertDistance(RADIUS, 1.5 * CENTERDISTANCE), true);
		leftMotor.rotate(-convertDistance(RADIUS, 1.5 * CENTERDISTANCE), false);

		// Set the wheel's rotation speed to ROTATESPEED
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);

		// Rotate the robot by 90 degrees
		leftMotor.rotate(convertAngle(RADIUS, TRACK, 90), true);
		rightMotor.rotate(-convertAngle(RADIUS, TRACK, 90), false);

		// Move forward, and return when a black line is detected
		detectBlackLine();

		// Move the robot backwards 1.5 * its center distance
		leftMotor.rotate(-convertDistance(RADIUS, 1.5 * CENTERDISTANCE), true);
		rightMotor.rotate(-convertDistance(RADIUS, 1.5 * CENTERDISTANCE), false);

		// Rotate the robot by 90 degrees
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);

		// Rotate the robot by -90 degrees
		leftMotor.rotate(-convertAngle(RADIUS, TRACK, 90), true);
		rightMotor.rotate(convertAngle(RADIUS, TRACK, 90), false);
	}

	/**
	 * This method makes the robot move forward. While moving, get the value
	 * returned by the light sensor, compute the derivative, and if the value of
	 * the derivative (diff) is than the threshold, a black line is detected.
	 * Beep and break out of the loop when the black line is detected.
	 */
	private void detectBlackLine() {

		leftMotor.setSpeed(FORWARDSPEED);
		rightMotor.setSpeed(FORWARDSPEED);

		leftMotor.forward();
		rightMotor.forward();

		while (leftMotor.isMoving() && rightMotor.isMoving()) {
			// fetching the values from the color sensor
			colorSensorValue.fetchSample(colorSensorData, 0);

			// getting the value returned from the sensor, and multiply it by
			// 1000 to scale
			float value = colorSensorData[0] * 1000;

			// computing the derivative at each point
			float diff = value - oldValue;

			// storing the current value, to be able to get the derivative on
			// the next iteration
			oldValue = value;
			if (diff < derivativeThreshold) {
				Sound.beep();
				break;
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
