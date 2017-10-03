package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class NavigationObstacle extends Thread implements UltrasonicController {
 
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor , sensorMotor;

	private final double RADIUS = 2.1;//2.093;
	private final double TRACK = 11.78;//11.9
	private static double Tile_Length = 30.48;

	private static final int FWSPEED = 150;
	private static final int DTSPEED = 50;
	private static final int RTSPEED = 50;
	private static final int MTRACCEL = 50;

	private static boolean isWallFollowing = false;
	
	private static final int FILTER_OUT = 20;


	private int distanceUS;
	private int filterControl;
	private final int backwardControl = 5;

	private final int bandCenter;
	private float distanceError;
	
	private double blockAngle = 0;
	
	private int[][] wayPoints = {{2, 1}, {1 , 1}, {1 , 2}, {2 , 0}};
	
	private int count = 0;

	public NavigationObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,EV3LargeRegulatedMotor sensorMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.bandCenter= 13;
	}
	
	@Override
	public int readUSDistance() {
		return this.distanceUS;
	}

	@Override
	public void processUSData(int distance) {
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distanceUS = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distanceUS = distance;
		}
		// process a movement based on the us distance passed in (BANG-BANG style)
		//here we get the distance of the sensor positioned at a 45 degrees angle

		// Supposed to be root 2 but sensor is not exactly 45 degrees
		// so we take an arbitrary value called anglePosition
		float distanceRoot = (float) distance;
				
		//calculate the error of the robot by subtracting the actual distance to the wanted distance
		this.distanceError = bandCenter - distanceRoot;
	}

	public void run(){

		leftMotor.stop();

		rightMotor.stop();
		
		sensorMotor.stop();
		sensorMotor.setAcceleration(MTRACCEL);
	
//		travelTo(1*Tile_Length , 0*Tile_Length);
//		travelTo(2*Tile_Length , 1*Tile_Length);
//		travelTo(2*Tile_Length , 2*Tile_Length);
//		travelTo(0*Tile_Length , 2*Tile_Length);
//		travelTo(1*Tile_Length , 1*Tile_Length);
		
		for(count = 0 ; count < wayPoints.length ; count++){
			travelTo(wayPoints[count][0]*Tile_Length, wayPoints[count][1]*Tile_Length);
		}
	}
	
	private void travelTo(double x , double y){
		double deltaX = x - odometer.getX();
		double deltaY = y - odometer.getY();

		double minimumAngle =  Math.atan2( deltaX , deltaY) - odometer.getTheta();

		turnTo(minimumAngle);

		// calculate the distance to next point
		double distance  = Math.hypot(deltaX, deltaY);


		// move to the next point
		leftMotor.setSpeed(FWSPEED);
		rightMotor.setSpeed(FWSPEED);
		leftMotor.rotate(convertDistance(RADIUS,distance), true);
		rightMotor.rotate(convertDistance(RADIUS, distance), true);

		while( (rightMotor.isMoving() && leftMotor.isMoving()) ){

			if((distanceUS < bandCenter)&&(!isWallFollowing)){
				
				blockAngle = odometer.getTheta();
				
				isWallFollowing = true;
				leftMotor.stop(true);
				rightMotor.stop(true);
				leftMotor.rotate(convertAngle(RADIUS, TRACK, 90), true);
				rightMotor.rotate(-convertAngle(RADIUS, TRACK, 90), false);
				sensorMotor.rotate(-90);
				
			}


			if(isWallFollowing){
				if (distanceError >= 0 && distanceError < bandCenter/2 + backwardControl){

					rightMotor.setSpeed(FWSPEED-DTSPEED);
					leftMotor.setSpeed(FWSPEED+DTSPEED);
					rightMotor.forward();
					leftMotor.forward();
				}

				else if(distanceError >= bandCenter/2 - backwardControl ){
					rightMotor.setSpeed(FWSPEED+DTSPEED);
					leftMotor.setSpeed(FWSPEED+DTSPEED);
					rightMotor.backward();
					leftMotor.backward();
				}

				else if (distanceError < 0){
					leftMotor.setSpeed(FWSPEED-DTSPEED);
					rightMotor.setSpeed(FWSPEED+DTSPEED);
					rightMotor.forward();
					leftMotor.forward();
				}
			}
			
			if(isWallFollowing && ( blockAngle - odometer.getTheta() > Math.PI/4)){
				leftMotor.stop();
				rightMotor.stop();
				sensorMotor.rotate(90, true);
				
				isWallFollowing = false;
				
				count--;
				break;
			}
		}	


		leftMotor.stop(true);
		rightMotor.stop(true);
	}

	private void turnTo(double theta) {
		leftMotor.setSpeed(RTSPEED);
		rightMotor.setSpeed(RTSPEED);

		if(theta > Math.PI){
			theta -= 2*Math.PI;

		}
		else if(theta<=-Math.PI){
			theta += 2*Math.PI;
		}

		if(theta < 0) { 
			leftMotor.rotate(-convertAngle(RADIUS, TRACK, -(theta*180)/Math.PI), true);
			rightMotor.rotate(convertAngle(RADIUS, TRACK, -(theta*180)/Math.PI), false);
		} 
		else { 
			leftMotor.rotate(convertAngle(RADIUS, TRACK, (theta*180)/Math.PI), true);
			rightMotor.rotate(-convertAngle(RADIUS, TRACK, (theta*180)/Math.PI), false);
		}
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}


}
