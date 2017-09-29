/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.lab2;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private static final double TILE_LENGTH = 30.48;
	private Odometer odometer;
	
	private static final Port colorSampler = LocalEV3.get().getPort("S1");

	private SensorModes colosSamplerSensor = new EV3ColorSensor(colorSampler);
	private SampleProvider colorSensorValue = colosSamplerSensor.getMode("Red");

	private float[] colorSensorData = new float[colosSamplerSensor.sampleSize()];

	private float oldValue = 0;
	
	private int counterX;
	private int counterY;
	
	private double theta;
	
	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		while (true) {
			correctionStart = System.currentTimeMillis();
			
			//fetching the values from the color sensor
			colorSensorValue.fetchSample(colorSensorData, 0);
			
			//getting the value returned from the sensor, and multiply it by 1000 to scale
			float value = colorSensorData[0]*1000;
			
			//computing the derivative at each point
			float diff = value - oldValue;
			
			//storing the current value, to be able to get the derivative on the next iteration
			oldValue = value;
			
			//if the derivative value at a given point is less than -50, this means that a black line is detected
			if(diff < -50) {
				
				//robot beeps
				Sound.beep();
				
				//get the status of the robot by the counter used in squareDriver, this counter keeps track of the orientation of the robot
				//int status = SquareDriver.getSquareCount();
				
				theta = odometer.getTheta() * 180 / Math.PI;
				/*
				 * The Y and X counter keeps track of how many horizontal and vertical black lines the robot detected respectively.
				 * This counter is used to computer the Y and X values respect to the origin (first intersection).
				 * theta tells us if the robot is moving forward horizontally or vertically.
				 */
				
			/*	if(status == 0){
					odometer.setY(counterY * TILE_LENGTH);
					counterY ++;
				}else if(status == 1 ){
					odometer.setX(counterX * TILE_LENGTH);
					counterX++;
				}else if(status == 2){
					counterY--;
					odometer.setY(counterY * TILE_LENGTH);					
				}else if(status == 3){
					counterX--;
					odometer.setX(counterX * TILE_LENGTH);
				}	*/
				
				if( (theta <= 360 && theta >= 315) || (theta >=0 && theta <= 45)){
					odometer.setY(counterY * TILE_LENGTH);
					counterY++;
				}else if(theta > 45 && theta <= 135){
					odometer.setX(counterX * TILE_LENGTH);
					counterX++;
				}else if(theta > 135 && theta <= 225){
					counterY--;
					odometer.setY(counterY * TILE_LENGTH);
				}else if(theta > 225 && theta < 315){
					counterX--;
					odometer.setX(counterX * TILE_LENGTH);
				}
				
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}

			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
	
	
	
}
