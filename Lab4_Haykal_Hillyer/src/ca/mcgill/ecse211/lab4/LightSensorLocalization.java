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
	private static final double TRACK = 11.9;
	private static final double TILE_LENGTH = 30.48;
	
	private static final int FORWARDSPEED = 150;
	private static final int ROTATESPEED = 50;
	private static final int DIFFERENCESPEED = 50;
	private static final int ACCELERATION = 50;
	private static final int horizontalConstant = 40;
	private static final int noiseMargin = 2;
	private static final double CENTERDISTANCE = 14;
	
	private static final int LIGHT_SENSOR_FILTER = 4;
	
	//data
	private int filterCounter = 0;
	private float oldValue = 0;
	private int derivativeThreshold = -50;
	private double actualTheta;
	private int angleRange = 45;
	private int lineCounter = 0;
	private double xminus, xplus, yminus, yplus;
	private double thetax, thetay;
	private double x, y;
	private double deltaThetaY;
	private double deltaThetaX;
	private double deltaTheta;
	
	public LightSensorLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer){
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}
	
	public void run(){
		Button.waitForAnyPress();
		
		System.out.println("Start light sensor localization");
		
		leftMotor.stop();
		leftMotor.setAcceleration(ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(ACCELERATION);
		
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);
		
		leftMotor.rotate(convertAngle(RADIUS, TRACK, 360), true);
		rightMotor.rotate(-convertAngle(RADIUS, TRACK, 360), true);
		
		while(leftMotor.isMoving() && rightMotor.isMoving()){
			//fetching the values from the color sensor
			colorSensorValue.fetchSample(colorSensorData, 0);
			
			//getting the value returned from the sensor, and multiply it by 1000 to scale
			float value = colorSensorData[0]*1000;
			
			//computing the derivative at each point
			float diff = value - oldValue;
			
			//storing the current value, to be able to get the derivative on the next iteration
			oldValue = value;
			
			System.out.println("diff: " + diff);
			
			if(diff < derivativeThreshold && filterCounter == 0){
				Sound.beep();
				filterCounter++;
				lineCounter++;
				
				if(lineCounter == 1){
					xminus = odometer.getTheta();
					
				}else if(lineCounter == 2){
					yplus = odometer.getTheta();
					
				}else if(lineCounter == 3){
					xplus = odometer.getTheta();
					
				}else if(lineCounter == 4){
					yminus = odometer.getTheta();
					
				}
			}else if(diff < derivativeThreshold && filterCounter > 0){
				filterCounter++;
			}else if(diff > derivativeThreshold){
				filterCounter = 0;
			}
			
		}
		
		
		thetay = yminus - yplus;
		thetax = xplus - xminus;
		
		x = - CENTERDISTANCE * Math.cos(thetay / 2.0);
		y = - CENTERDISTANCE * Math.cos(thetax / 2.0);
		deltaThetaY = ( Math.PI / 2.0 ) - yminus + Math.PI + ( thetay / 2.0 );
		deltaThetaX = Math.PI - (thetax / 2.0) - xplus;
		
		odometer.setX(x);
		odometer.setY(y);
		odometer.setTheta(odometer.getTheta() + deltaThetaY);
		
		Navigation navigation = new Navigation(odometer, leftMotor, rightMotor);
		navigation.travelTo(0, 0);
		
		System.out.println("xminus: " + xminus * 180 / Math.PI);
		System.out.println("yplus: " + yplus* 180 / Math.PI);
		System.out.println("xplus: " + xplus* 180 / Math.PI);
		System.out.println("yminus: " + yminus* 180 / Math.PI);
		System.out.println("x odometer: " + odometer.getX());
		System.out.println("y odometer: " + odometer.getY());
		System.out.println("theta odometer: " + odometer.getTheta()* 180 / Math.PI);
		System.out.println("x: " + x);
		System.out.println("y: " + y);
		System.out.println("deltaThetaY: " + deltaThetaY* 180 / Math.PI);
		System.out.println("new theta: " + (odometer.getTheta() + deltaThetaY) * 180 / Math.PI);
		System.out.println("deltaThetaX; " + deltaThetaX * 180 / Math.PI);
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}
}
