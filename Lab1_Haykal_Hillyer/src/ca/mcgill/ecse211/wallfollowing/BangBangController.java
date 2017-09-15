package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private static final int FILTER_OUT = 20;
	private int filterControl;

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;

	// Distance constants
	private static final int tinyRangeDist = 10;
	private static final int minRangeDist = 20;
	private static final int maxRangeDist = 30;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		// this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		// EJ
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		float floatDistance = this.distance / (float) 1.4;

		this.distance = (int) floatDistance;

		if (this.distance > bandCenter + bandwidth) { // Too far from wall on left

			WallFollowingLab.leftMotor.setSpeed(motorLow); // Turn Left
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();

		} else if (this.distance < bandCenter - bandwidth) { // Too close to wall on left

			WallFollowingLab.leftMotor.setSpeed(motorHigh); // Turn Right
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		// End EJ
		/*else if( this.distance < 10) {
			
			WallFollowingLab.leftMotor.setSpeed(motorHigh); // Turn Right
			WallFollowingLab.rightMotor.setSpeed(0); // Sharper turn
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
			
		}*/

		/*
		 * // TODO: there are some errors when it gets within the minimum range that the
		 * // sensor can handle, the "dead width" where it will run into the wall if
		 * (minRangeDist < distance && distance < maxRangeDist) { // Robot is in the
		 * acceptable range
		 * 
		 * WallFollowingLab.leftMotor.setSpeed(motorHigh);
		 * WallFollowingLab.rightMotor.setSpeed(motorHigh);
		 * WallFollowingLab.leftMotor.forward(); WallFollowingLab.rightMotor.forward();
		 * 
		 * } else if (distance < tinyRangeDist) { // Robot way too close to the wall on
		 * the left
		 * 
		 * WallFollowingLab.leftMotor.setSpeed(motorHigh);// Turn right
		 * WallFollowingLab.rightMotor.setSpeed(0); // Stop motor for sharper turn
		 * WallFollowingLab.leftMotor.forward(); WallFollowingLab.rightMotor.forward();
		 * 
		 * } else if (distance < minRangeDist) { // Robot too close to the wall on the
		 * left
		 * 
		 * WallFollowingLab.leftMotor.setSpeed(motorHigh);// Turn right
		 * WallFollowingLab.rightMotor.setSpeed(motorLow);
		 * WallFollowingLab.leftMotor.forward(); WallFollowingLab.rightMotor.forward();
		 * 
		 * } else if (maxRangeDist < distance) { // Robot too far from the wall on the
		 * left
		 * 
		 * WallFollowingLab.leftMotor.setSpeed(motorLow);// Turn left
		 * WallFollowingLab.rightMotor.setSpeed(motorHigh);
		 * WallFollowingLab.leftMotor.forward(); WallFollowingLab.rightMotor.forward();
		 * 
		 * }
		 */
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
