//Brandon Clark 720474282
//Meggie Cruser 720463647

import java.awt.Point;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.Sound;

public class Assignment4 {

	//Motors
	static EV3MediumRegulatedMotor mA = new EV3MediumRegulatedMotor(MotorPort.A);
	static EV3MediumRegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B);

	static EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(SensorPort.S2);
	static SensorMode d = (SensorMode) ultra.getDistanceMode();
	static float[] sample = new float[d.sampleSize()];

	//Front button
	static EV3TouchSensor frontButton = new EV3TouchSensor(SensorPort.S3);

	//Sensor Setup
	static SensorMode touch2 = frontButton.getTouchMode();
	static float[] sampleFront = new float[touch2.sampleSize()];
	
	static double orientation = 0.000000;
	
	static double wheelbase = 11.9;
	
	static double start;
	
	static double x = 0, y = 0, hitx, hity, hittime;
	static int baseSpeeed = 300;
	static int scale = 3000;
	static int lspeeed = 100, rspeeed = 100;
	static double distance = getDistance();
	static double oldDistance = 15.0;
	static double deltaD = 0;

	
	public static void main(String[] args) throws InterruptedException {

		//Beep
		beep();
		System.out.println("TOUCH ME");
		waitForButtonPress();
			

		while(!atGoal()) {
			//Follow the mline
			while(!getBumper() && !atGoal()) {
				followMLine();
//				System.out.println("c: " + Double.toString(x).concat("000000000000000000").substring(0, 4) + " " + Double.toString(y).concat("000000000000000000").substring(0, 4));
			}
			if(atGoal()) break;
			//If we hit something
			//Turn and follow it			
			//While not on the mline
			while(distanceToMLine() < 2) {
				followWall();
//				System.out.println("a: " + Double.toString(x).concat("000000000000000000").substring(0, 4) + " " + Double.toString(y).concat("000000000000000000").substring(0, 4));
			}
			
			while(distanceToMLine() > 2) {
				followWall();
//				System.out.println("b: " + Double.toString(x).concat("000000000000000000").substring(0, 4) + " " + Double.toString(y).concat("000000000000000000").substring(0, 4));
			}
		}
		
		beep();
		stop();
		end();

	}
	
	private static boolean atGoal() {
		if(x > 75 && x < 85 && y < 185 && y > 175) {
			return true;
		} else {
			return false;
		}
	}
	
	private static double distanceToMLine() {
		return Math.abs((Math.abs(-1*y + 2.25*x)/(Math.sqrt(1+2.25*2.25))));
	}
	
	private static void followMLine() throws InterruptedException {
		//Turn to be parallel to m line
		int angle = 0;
		if(y > 180) {
			angle = -154;
		} else {
			angle = 24;
		}
		while(orientation > angle + 2) {
			start = System.currentTimeMillis();
			move(100, 100, false, true, 50);
		}
		
		while(orientation < angle - 2) {
			start = System.currentTimeMillis();
			move(100, 100, true, false, 50);
		}
		
		start = System.currentTimeMillis();
		move(200, 200, true, true, 100);
	}
	
	private static void followWall() throws InterruptedException {
		
		start = System.currentTimeMillis();

		distance = getDistance();
		deltaD = (oldDistance - distance);
		
		if(!Double.isFinite(deltaD)) {
			lspeeed = 50;
			rspeeed = baseSpeeed;
		} else if((deltaD > 0) || distance < .1) { //turning right 
			lspeeed = (int) (baseSpeeed + ( deltaD * scale));
			rspeeed = baseSpeeed;
		} else if((deltaD <  0) || distance > .25) { //turning left 
			lspeeed = baseSpeeed;
			rspeeed = (int) (baseSpeeed + ( -1* deltaD * scale));
		} else {
			lspeeed = baseSpeeed;
			rspeeed = baseSpeeed;
		}
		
		if (lspeeed > baseSpeeed + 100) {
			lspeeed = baseSpeeed + 100; 
		}
		if (rspeeed > baseSpeeed + 100) {
			rspeeed = baseSpeeed + 100; 
		}
		if (lspeeed < baseSpeeed - 100) {
			lspeeed = baseSpeeed - 100; 
		}
		if (rspeeed < baseSpeeed - 100) {
			rspeeed = baseSpeeed - 100; 
		}
		
		if(distanceToMLine() < 20) {
			lspeeed = lspeeed/2;
			rspeeed = rspeeed/2;
		}
		
		if (getBumper() == true) {
			System.out.println("\nO: " + Double.toString(orientation).concat("000000000000000000").substring(0, 4));

			start = System.currentTimeMillis();
			move(100, 100, false, false, 3000);
			stop();
			
			start = System.currentTimeMillis();
			move(180, 180, true, false, 1000);
			oldDistance = getDistance();
			stop();
			
			System.out.println("\nO: " + Double.toString(orientation).concat("000000000000000000").substring(0, 4));

		}
		
		move(lspeeed, rspeeed, true, true, 100);
	}
	
	private static void move(int aSpeed, int bSpeed, boolean directionA, boolean directionB, double time) throws InterruptedException {
		mA.setSpeed(aSpeed);
		mB.setSpeed(bSpeed);
		if(directionA) {
			mA.forward();
		} else {
			mA.backward();
		}
		if(directionB) {
			mB.forward();
		} else {
			mB.backward();
		}
		Thread.sleep((long) time);
		double a = (Math.PI*5.6) * (aSpeed / 360.0); //a is the velocity
		if(!directionA) {
			a = -1.0*a;
		}
		double b = (Math.PI*5.6) * (bSpeed / 360.0); //b is the other velocity 
		if(!directionB) {
			b = -1.0*b;
		}
		
		double elapsed = System.currentTimeMillis() - start;
		
		double deltaO = (((aSpeed*2.8*(a/Math.abs(a)))-(bSpeed*2.8*(b/Math.abs(b))))/wheelbase)*(elapsed/1000.0);
		//double deltaO = ((a-b)/wheelbase)*(elapsed/1000.0);
		
//		System.out.println(orientation);
		orientation += deltaO;
		x += (1.0/2.0)*(Math.sin(Math.toRadians(orientation)))*(a+b)*(elapsed/1000.0);
		y += (1.0/2.0)*(Math.cos(Math.toRadians(orientation)))*(a+b)*(elapsed/1000.0);
	}
	
	private static void stop() {
		mA.stop();
		mB.stop();
	}
	
	private static float getDistance() {
		d.fetchSample(sample, 0);
		return sample[0];
	}
	
	private static boolean getBumper() {
		touch2.fetchSample(sampleFront, 0);
		return sampleFront[0] == 1;
	}

	private static void end() {
		while(mA.isMoving() || mB.isMoving()) {
		}
		frontButton.close();
		mA.close();
		mB.close();
		ultra.close();
	}

	private static void beep() {
		lejos.hardware.Sound.setVolume(10);
		lejos.hardware.Sound.beep();
	}

	private static void waitForButtonPress() throws InterruptedException {
//		System.out.println("Waiting for button press");
		Button.ENTER.waitForPress();
		Thread.sleep(200);
	}
}

