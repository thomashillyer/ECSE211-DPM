package ca.mcgill.ecse211.lab3;

public class Navigation {
	public void travelTo(double x, double y) {
		// This method causes the robot to travel to the absolute field location (x, y),
		// specified in tile points.This method should continuously call turnTo(double
		// theta) and then set the motor speed to forward(straight). This will make sure
		// that your heading is updated until you reach your exact goal. This method
		// will poll the odometer for information
	}

	public void turnTo(double theta) {
		// this method should turn (on point) to the absolute heading theta, by using a
		// minimal angle
	}

	public boolean isNavigating() {
		// method returns true if another thread has called travelTo() or turnTo() and
		// the method has yet to return; false otherwise

		return false;
	}
}
