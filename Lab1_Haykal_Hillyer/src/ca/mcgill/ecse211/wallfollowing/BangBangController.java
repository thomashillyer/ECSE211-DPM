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
	private int distanceSum;
	private int averageDistance;
	private int index;


	private static final int tinyRangeDist = 10;
	private static final int minRangeDist = 20;
	private static final int maxRangeDist = 30;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.filterControl = 0;
		this.distanceSum = 0; 
		this.index = 0;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		//this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		if (distance >= 50 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 50) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			//index++; 
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			//index++;
			this.distance = distance;
		}		


		//float floatDistance = this.distance / (float)1.4;

		//this.distance = (int) floatDistance;

		//distanceSum += this.distance;

		//averageDistance = distanceSum / index;

		//this.distance = averageDistance;

		if(this.distance > bandCenter - bandwidth && this.distance < bandCenter + bandwidth){
			WallFollowingLab.leftMotor.setSpeed(motorHigh); 
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		else if(this.distance > bandCenter + bandwidth){
			WallFollowingLab.leftMotor.setSpeed(motorLow); //turn left 
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}else if (this.distance < 10){

			WallFollowingLab.leftMotor.setSpeed(50); //hard right turn
			WallFollowingLab.rightMotor.setSpeed(100);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.backward();

		} 
		else if(this.distance <  bandCenter - bandwidth){
			WallFollowingLab.leftMotor.setSpeed(motorHigh); //turn right
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		/*
		  // TODO: there are some errors when it gets within the minimum range that the
		  // sensor can handle, the "dead width" where it will run into the wall 
		if(minRangeDist < distance && distance < maxRangeDist) { // Robot is in the


		  WallFollowingLab.leftMotor.setSpeed(motorHigh);
		  WallFollowingLab.rightMotor.setSpeed(motorHigh);
		  WallFollowingLab.leftMotor.forward(); WallFollowingLab.rightMotor.forward();

		  } else if (distance < tinyRangeDist) { // Robot way too close to the wall on


		  WallFollowingLab.leftMotor.setSpeed(motorHigh);// Turn right
		  WallFollowingLab.rightMotor.setSpeed(0); // Stop motor for sharper turn
		  WallFollowingLab.leftMotor.forward(); WallFollowingLab.rightMotor.forward();

		  } else if (distance < minRangeDist) { // Robot too close to the wall on the


		  WallFollowingLab.leftMotor.setSpeed(motorHigh);// Turn right
		  WallFollowingLab.rightMotor.setSpeed(motorLow);
		  WallFollowingLab.leftMotor.forward(); WallFollowingLab.rightMotor.forward();

		  } else if (maxRangeDist < distance) { // Robot too far from the wall on the


		  WallFollowingLab.leftMotor.setSpeed(motorLow);// Turn left
		  WallFollowingLab.rightMotor.setSpeed(motorHigh);
		  WallFollowingLab.leftMotor.forward(); WallFollowingLab.rightMotor.forward();

		  }
		 */

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
