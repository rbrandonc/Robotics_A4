//Brandon Clark 720474282
//Meggie Cruser 720463647

import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.Sound;

public class Assignment2 {

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
	
	static float orientation = 0;
	
	static double wheelbase = 11.9;
	
	public static void main(String[] args) throws InterruptedException {
		//Beep
		beep();
		System.out.println("TOUCH ME");
		waitForButtonPress();
//
		//Drive to wall
		while(getBumper() != true) {
			move(350, 350, true, true, 100);
		}
		stop();
		
		//Back up
		move(100, 100, false, false, 3000);
		stop();
		
		//Turn
		move(180, 180, true, false, 1000);
		
		stop();
		
		int baseSpeeed = 100;
		int scale = 3000;
		int lspeeed = 100, rspeeed = 100;
		double distance = getDistance();
		double oldDistance = getDistance();
		double deltaD = 0;
		
		//Spidey senses
		while(orientation > 0 ) {
			//System.out.println("\n\n\n\n\n\n\n\n");

			distance = getDistance();
			deltaD = (oldDistance - distance);
			
			if(!Double.isFinite(deltaD)) {
				lspeeed = 20;
				rspeeed = baseSpeeed;
			} else if((deltaD > 0) || distance < .1) { //turning right 
				lspeeed = (int) (baseSpeeed + ( deltaD * scale));
				rspeeed = baseSpeeed;
			} else if((deltaD <  0) || distance > .25) { //turning left 
				lspeeed = baseSpeeed;
				rspeeed = (int) (baseSpeeed + ( -1* deltaD * scale));
			} else {
				//int x = lspeeed;
				lspeeed = baseSpeeed;
				rspeeed = baseSpeeed;
			}
			
			if (lspeeed > 1000) {
				lspeeed = 100; 
			}
			if (rspeeed > 1000) {
				rspeeed = 100; 
			}
			
			System.out.print(lspeeed + " ");
			System.out.print(rspeeed + " ");
			System.out.print(distance + " " + oldDistance);
			System.out.print(deltaD + " ");
			System.out.println();

			move(lspeeed, rspeeed, true, true, 1);
		}

		
		//Move 75cm
		move(130, 130, true, true, 0);
		mA.rotate(1504, true);
		mB.rotate(1504, true);

		end();

	}
	
	private static void move(int aSpeed, int bSpeed, boolean directionA, boolean directionB, double time) throws InterruptedException {
		long start = System.currentTimeMillis();
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
		double a = 2.857 * aSpeed;
		if(!directionA) {
			a = -1*a;
		}
		double b = 2.857 * bSpeed;
		if(!directionB) {
			b = -1*b;
		}
		
		double elapsed = System.currentTimeMillis() - start;
		orientation += (((a-b)/wheelbase)*(elapsed/1000));
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
//		mario();
		while(mA.isMoving() || mB.isMoving()) {
		}
		frontButton.close();
		mA.close();
		mB.close();
		ultra.close();
	}

	private static void beep() {
		lejos.hardware.Sound.beep();
	}

	private static void waitForButtonPress() throws InterruptedException {
//		System.out.println("Waiting for button press");
		Button.ENTER.waitForPress();
		Thread.sleep(200);
	}
	
	private final static int C4 = 262;
	private final static int D4 = 294;
	private final static int E4 = 330;
	private final static int F4 = 349;
	private final static int G4 = 392;
	private final static int A4 = 440;
	private final static int As4 = 466;
	private final static int B4 = 494;

	private final static int C5 = 523;
	private final static int Cs5 = 554;
	private final static int D5 = 587;
	private final static int Ds5 = 622;
	private final static int E5 = 659;
	private final static int F5 = 698;
	private final static int Fs5 = 740;
	private final static int G5 = 784;
	private final static int A5 = 880;

	private final static int C6 = 1047;

	private static void mario() {
		int vol = Sound.getVolume();
		System.out.println("Mario!");
		Sound.setVolume(66);

		play(E5, 100, 150);
		play(E5, 100, 300);
		play(E5, 100, 300);
		play(C5, 100, 100);
		play(E5, 100, 300);
		play(G5, 100, 550);
		play(G4, 100, 575);

		for (int i=0; i<2; i++) {
			play(C5, 100, 450);
			play(G4, 100, 400);
			play(E4, 100, 500);
			play(A4, 100, 300);
			play(B4, 80, 330);
			play(As4, 100, 150);
			play(A4, 100, 300);
			play(G4, 100, 200);
			play(E5, 80, 200);
			play(G5, 50, 150);
			play(A5, 100, 300);
			play(F5, 80, 150);
			play(G5, 50, 350);
			play(E5, 80, 300);
			play(C5, 80, 150);
			play(D5, 80, 150);
			play(B4, 80, 500);
		}

		
		play(B4, 100, 300); //F
		
		play(G5, 100, 100);
		play(Fs5, 100, 150);
		play(E5, 100, 150);
		play(Ds5, 150, 300);
		play(E5, 150, 300);
		play(G4, 100, 150);
		play(A4, 100, 150);
		play(B4, 100, 300);
		play(A4, 100, 150);
		play(B4, 100, 100);
		play(D5, 100, 225);
		play(B4, 100, 300);
		play(G5, 100, 100);
		play(Fs5, 100, 150);
		play(E5, 100, 150);
		play(Ds5, 150, 300);
		play(E5, 200, 300);
		play(C6, 80, 300);
		play(C6, 80, 150);
		play(C6, 80, 300);
		play(G4, 100, 300);
		play(B4, 100, 300);
		// Chorus 2:
		play(G5, 100, 100);
		play(Fs5, 100, 150);
		play(E5, 100, 150);
		play(Ds5, 150, 300);
		play(E5, 150, 300);
		play(G4, 100, 150);
		play(A4, 100, 150);
		play(B4, 100, 300);
		play(A4, 100, 150);
		play(B4, 100, 100);
		play(D5, 100, 425);
		// End 2
		play(D5, 100, 450);
		play(Cs5, 100, 425);
		play(B4, 100, 350);
		play(G4, 100, 300);
		play(B4, 100, 300);
		play(B4, 100, 150);
		play(B4, 100, 300);
		play(B4, 100, 300);
		// Insert 2
		
		

		System.out.println("End.");
		Sound.setVolume(vol);
		Button.ESCAPE.waitForPressAndRelease();
	}

	private static void play(int freq, int dur, int delay) {
		if (100 <= freq  && freq  <= 12000 &&
			10  <= dur   && dur   <= 10000 &&
			10  <= delay && delay <= 10000) {
			Sound.playTone(freq, dur);
			try {
				Thread.sleep(delay/2);
			}
			catch (Exception e) {}
		}
	}
	
}

//double distanceToWall = getDistance();
//double oldDistanceToWall;
//
//while(orientation > 0) {
//	oldDistanceToWall = distanceToWall;
//	
//	//Move forwards 4.48855050383cm in .5s
//	move(180, 180, true, true, 500);
//	
//	distanceToWall = getDistance();
//	
//	//Positive means we got farther away, negative means we got closer
//	double changeInDistanceToWall = (oldDistanceToWall - distanceToWall);
//	double degreesChange = 57.2958*Math.atan((changeInDistanceToWall*100) / 4.48855050383); //1.905
//	double newOrientation;
//	if(degreesChange > 0)
//		newOrientation = orientation + degreesChange;
//	else
//		newOrientation = orientation + degreesChange;
//	
//	if(distanceToWall > .5) {
//		newOrientation = orientation - 10;
//	} 
//	System.out.println("\n\n\n\n\n\nOld D: " + oldDistanceToWall);
//	System.out.println("New D: " + distanceToWall);
//	System.out.println("Orien: " + orientation);
//	System.out.println("Chang: - " + degreesChange);
//	System.out.println("New  : - " + newOrientation);
//	
//		//turn to face new orientation
//		if(degreesChange > 0) {
//			while(orientation < newOrientation) {
//				move(100, 100, true, false, 100);
//			}
//		}
//		
//		if(degreesChange < 0 && distanceToWall > .2) {
//			while(orientation > newOrientation) {
//				move(100, 100, false, true, 100);
//			}
//		}
//
//	
//	stop();
////	waitForButtonPress();
//
//	Thread.sleep(200);
//	
//	//Re measure distance
//	distanceToWall = getDistance();
//	
//}
//stop();
