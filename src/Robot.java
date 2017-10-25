import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;

public class Robot {
	//Motors
			EV3MediumRegulatedMotor mA = new EV3MediumRegulatedMotor(MotorPort.A);
			EV3MediumRegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B);

			EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(SensorPort.S2);
			SensorMode distance = (SensorMode) ultra.getDistanceMode();
			float[] sample = new float[distance.sampleSize()];

			//Front button
			EV3TouchSensor frontButton = new EV3TouchSensor(SensorPort.S3);

			//Sensor Setup
			SensorMode touch2 = frontButton.getTouchMode();
			float[] sampleFront = new float[touch2.sampleSize()];
}
