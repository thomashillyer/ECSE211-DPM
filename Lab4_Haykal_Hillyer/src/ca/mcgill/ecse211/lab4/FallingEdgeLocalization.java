package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Odometer;
import ca.mcgill.ecse211.lab4.UltrasonicController;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class FallingEdgeLocalization extends Thread implements UltrasonicController {


	private int distanceUS = 255;
	private int filterControl;
	private static final int FILTER_OUT = 20;


	private static final int FWSPEED = 150;
	private static final int RTSPEED = 50;
	private static final int MTRACCEL = 50;


	private final double RADIUS = 2.093;
	private final double TRACK = 11.9;
	private static double Tile_Length = 30.48;

	private final int EdgeConstante = 40;
	private final int MarginErrorK = 2;


	private boolean isFacingWall = false;

	private static double angleAlfa = 0.0;
	private static double angleBeta = 0.0;
	private static double deltaTheta = 0.0;



	private static Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor , sensorMotor;


	public FallingEdgeLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;

	}



	public void run(){

		leftMotor.stop();
		leftMotor.setAcceleration(MTRACCEL);

		rightMotor.stop();		
		rightMotor.setAcceleration(MTRACCEL);


		leftMotor.setSpeed(RTSPEED);
		rightMotor.setSpeed(RTSPEED);

		leftMotor.rotate(convertAngle(RADIUS, TRACK, 360), true);
		rightMotor.rotate(-convertAngle(RADIUS, TRACK, 360), true);

		while((rightMotor.isMoving() && leftMotor.isMoving())){

			if(!isFacingWall && (distanceUS <=EdgeConstante)) {	
				angleAlfa = odometer.getTheta();
				Sound.beep();
				isFacingWall = true;

			}else if(isFacingWall && (distanceUS > EdgeConstante)){
				angleBeta = odometer.getTheta();
				Sound.beep();
				isFacingWall = false;
			}


		}


		if(angleAlfa > angleBeta){
			deltaTheta = Math.PI /4.0 - (angleAlfa + angleBeta)/2.0 ;
		}else if(angleAlfa <= angleBeta){
			deltaTheta = 5.0*Math.PI /4.0 - (angleAlfa + angleBeta)/2.0 ;

		}

		System.out.println(deltaTheta);
		leftMotor.rotate(-convertAngle(RADIUS, TRACK, deltaTheta*180/Math.PI), true);
		rightMotor.rotate(convertAngle(RADIUS, TRACK, deltaTheta*180/Math.PI), false);

	}


	@Override
	public int readUSDistance() {
		return this.distanceUS;
	}

	@Override
	public void processUSData(int distance) {

		if(angleBeta != 0.0){
			distance = 255;
		}else {


			if (distance >= 255 && filterControl < FILTER_OUT) {
				// bad value, do not set the distance var, however do increment the
				// filter value
				filterControl++;
			} else if (distance >= 255) {
				// We have repeated large values, so there must actually be nothing
				// there: leave the distance alone

				distance = 255;

				this.distanceUS = distance;
			} else {
				// distance went below 255: reset filter and leave
				// distance alone.
				filterControl = 0;
				this.distanceUS = distance;

			}
		}
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}

}
