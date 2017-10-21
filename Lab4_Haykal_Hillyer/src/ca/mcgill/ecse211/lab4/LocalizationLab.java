package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LocalizationLab {

	// Ultrasonic sensor connected to port S2
	private static final Port usPort = LocalEV3.get().getPort("S1");
	// Left motor connected to output A
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	// Right motor connected to output D
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 16;

	public static void main(String[] args) {
		int buttonChoice;

		@SuppressWarnings("resource") // Because we don't bother to close this
										// resource
		// usSensor is the instance
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		// usDistance provides samples from this instance
		SampleProvider usDistance = usSensor.getMode("Distance");
		// usData is the buffer in which data are returned
		float[] usData = new float[usDistance.sampleSize()];

		final TextLCD screen = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, screen);

		FallingEdgeLocalization FallingLocalization = new FallingEdgeLocalization(leftMotor, rightMotor, odometer);
		RisingEdgeLocalization raisingLocalization = new RisingEdgeLocalization(leftMotor, rightMotor, odometer);
		LightSensorLocalization lightSensorLocalization = new LightSensorLocalization(leftMotor, rightMotor, odometer);

		do {
			// clear the display
			screen.clear();

			// ask the user to confirm the start
			screen.drawString("< Left   | Right >", 0, 0);
			screen.drawString("         |        ", 0, 1);
			screen.drawString(" falling | Rising ", 0, 2);
			screen.drawString("         |    	 ", 0, 3);
			screen.drawString("         |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			odometer.start();
			odometryDisplay.start();

			UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, FallingLocalization);
			usPoller.start();

			FallingLocalization.start();
			lightSensorLocalization.start();
		} else {
			odometer.start();
			odometryDisplay.start();

			UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, raisingLocalization);
			usPoller.start();

			raisingLocalization.start();
			lightSensorLocalization.start();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
