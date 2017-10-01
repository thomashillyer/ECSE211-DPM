package ca.mcgill.ecse211.wallfollowing;

public class BangBangController implements UltrasonicController {

	private static final int FILTER_OUT = 20;
	private int filterControl;

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.filterControl = 0;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		// this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

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

			float floatDistance = distance / (float) 1.4;

			this.distance = (int) floatDistance;
		}

		if (this.distance > bandCenter - bandwidth && this.distance < bandCenter + bandwidth) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		} else if (this.distance > 200) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh - 50);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		} else if (this.distance > bandCenter + bandwidth) {
			WallFollowingLab.leftMotor.setSpeed(motorLow); // turn left
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		} else if (this.distance < 10) {
			WallFollowingLab.leftMotor.setSpeed(50);
			WallFollowingLab.rightMotor.setSpeed(100);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.backward();

		} else if (this.distance < bandCenter - bandwidth) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
