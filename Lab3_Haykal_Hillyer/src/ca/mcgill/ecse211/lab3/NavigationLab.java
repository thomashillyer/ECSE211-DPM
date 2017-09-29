package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigationLab {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 11.5;

	public static void main(String[] args) {
		int buttonChoice;

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
		//OdometryCorrection odometryCorrection = new OdometryCorrection(odometer);

		do {
			// clear the display
			t.clear();

			// ask the user to confirm the start
			t.drawString("Press Right >", 0, 0);
			t.drawString("Start Navig > ", 0, 1);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_RIGHT); // can probably modify this and just start on any button press

		if (buttonChoice == Button.ID_RIGHT) {

			odometer.start();
			odometryDisplay.start();

			//odometryCorrection.start();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}
