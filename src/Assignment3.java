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

public class Assignment3 {

	//Motors
	static EV3MediumRegulatedMotor mA = new EV3MediumRegulatedMotor(MotorPort.A);
	static EV3MediumRegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B);

	static EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(SensorPort.S2);
	static SensorMode distance = (SensorMode) ultra.getDistanceMode();
	static float[] sample = new float[distance.sampleSize()];

	//Front button
	static EV3TouchSensor frontButton = new EV3TouchSensor(SensorPort.S3);

	//Sensor Setup
	static SensorMode touch2 = frontButton.getTouchMode();
	static float[] sampleFront = new float[touch2.sampleSize()];
	
	static double orientation = 0.000000;
	
	static double wheelbase = 11.9;
	
	static double start;
	
	static double x = 0, y = 0, hitx, hity, hittime;
	
	public static void main(String[] args) throws InterruptedException {
		//Beep
		beep();
		System.out.println("TOUCH ME");
		waitForButtonPress();
//
		//Drive to wall
		while(getBumper() != true) {
			start = System.currentTimeMillis();
			move(360, 360, true, true, 100);
//			stop();
//			waitForButtonPress();
		}
		
		
		stop();
		beep(); 
		
		//Back up
		start = System.currentTimeMillis();
		move(100, 100, false, false, 3000);
		stop();
		
		hitx = x;
		hity = y;
		
		//Turn
		start = System.currentTimeMillis();
		move(180, 180, true, false, 1000);
		
		stop();
		hittime = System.currentTimeMillis();

		
		int baseSpeeed = 100;
		int scale = 3000;
		int lspeeed = 100, rspeeed = 100;
		double distance = getDistance();
		double oldDistance = getDistance();
		double deltaD = 0;
		
		//Spidey senses
		while(!atStart()) {
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
			
			if (lspeeed > 175) {
				lspeeed = 175; 
			}
			if (rspeeed > 175) {
				rspeeed = 175; 
			}
			if (lspeeed < 25) {
				lspeeed = 25; 
			}
			if (rspeeed < 25) {
				rspeeed = 25; 
			}
			
			if (getBumper() == true) {
				start = System.currentTimeMillis();
				move(100, 100, false, false, 3000);
				stop();
				
				start = System.currentTimeMillis();
				move(180, 180, true, false, 1000);
				
				stop();
			}
			
			move(lspeeed, rspeeed, true, true, 100);
		}

		//Move 75cm
//		move(130, 130, true, true, 0);
//		mA.rotate(1504, true);
//		mB.rotate(1504, true);
		beep();beep();beep();
		stop();
		
	while(orientation < -180)	{
		start = System.currentTimeMillis();
		move(180, 180, true, false, 100); 
	}
	stop();

	while (y > 0) {
		start = System.currentTimeMillis();
		move(180, 180, true, true, 100); 
	}
	
	stop(); 
	beep();beep();beep();beep();beep();beep();beep();beep();beep();beep();beep();

//		end();

	}
	
	private static boolean atStart() {
		if(System.currentTimeMillis() - hittime < 5000) {
			return false;
		}
		
		if(x > hitx-2 && x < hitx+2 && y < hity+10) {
			return true;
		} else {
			return false;
		}
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
		double a = (Math.PI*5.6) * (aSpeed / 360.0);
		if(!directionA) {
			a = -1.0*a;
		}
		double b = (Math.PI*5.6) * (bSpeed / 360.0);
		if(!directionB) {
			b = -1.0*b;
		}
		
		double elapsed = System.currentTimeMillis() - start;
		
		double deltaO = (((aSpeed*2.8*(a/Math.abs(a)))-(bSpeed*2.8*(b/Math.abs(b))))/wheelbase)*(elapsed/1000.0);

//		System.out.println(orientation);
		orientation += deltaO;
		x += (1.0/2.0)*(Math.sin(Math.toRadians(orientation)))*(a+b)*(elapsed/1000.0);
		y += (1.0/2.0)*(Math.cos(Math.toRadians(orientation)))*(a+b)*(elapsed/1000.0);
		System.out.println("\no: " + Double.toString(orientation));

	}
	
	private static void stop() {
		mA.stop();
		mB.stop();
	}
	
	private static float getDistance() {
		distance.fetchSample(sample, 0);
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

