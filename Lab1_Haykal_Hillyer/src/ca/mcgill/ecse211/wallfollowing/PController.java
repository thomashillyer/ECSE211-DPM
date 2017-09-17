package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int FILTER_OUT = 20;

	private static final double PROPORTION_CONST = 5;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;

	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {

		int diff, leftSpeed, rightSpeed;

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 150 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 150) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		if (this.distance <= 55) {
			float floatDistance = this.distance / (float) 1.4;

			this.distance = (int) floatDistance;
		}

		int distanceError = bandCenter - this.distance;

		// TODO: process a movement based on the us distance passed in (P style)
		if (Math.abs(distanceError) <= bandWidth) {
			leftSpeed = MOTOR_SPEED;
			rightSpeed = MOTOR_SPEED;
			WallFollowingLab.leftMotor.setSpeed(leftSpeed);
			WallFollowingLab.rightMotor.setSpeed(rightSpeed);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		} else if (distanceError > 0) {
			diff = calcCorrection(distanceError);
			leftSpeed = MOTOR_SPEED + diff;
			rightSpeed = MOTOR_SPEED - diff;
			WallFollowingLab.leftMotor.setSpeed(leftSpeed); // Turn Right
			WallFollowingLab.rightMotor.setSpeed(rightSpeed);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		} else if (distanceError < 0) {
			diff = calcCorrection(distanceError);
			leftSpeed = MOTOR_SPEED - diff;
			rightSpeed = MOTOR_SPEED + diff;
			WallFollowingLab.leftMotor.setSpeed(leftSpeed); // Turn Right
			WallFollowingLab.rightMotor.setSpeed(rightSpeed);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
	}

	private int calcCorrection(int errorVal) {
		int speedCorrection;
		if (errorVal < 0)
			errorVal = -errorVal;

		speedCorrection = (int) (PROPORTION_CONST * errorVal);

		if (speedCorrection >= MOTOR_SPEED)
			speedCorrection = 50;

		return speedCorrection;

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
