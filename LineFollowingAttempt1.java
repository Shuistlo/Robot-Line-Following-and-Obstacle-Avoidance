import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;

/*
 * This will help me to determint which values are returned for black, and which are returned for white. This should allow me to write a program that
 * will make the robot follow a straight black line. Turning implementation will be added in later.
 */

public class LineFollowingAttempt1 {

	public LineFollowingAttempt1() {
		//initialise colorSensor and set it to the correct port
		Brick brick = new BrickFinder.getDefault();
		Key escapeKey = brick.getKey("escpae");
		Port s1 = LocalEV3.get().getPort("S1");

		//configure the colorSensor and it's redMode
    	EV3ColorSensor colorSensor = new EV3ColorSensor(s1);
		SampleProvider redMode = colorSensor.getRedMode();
		float[] colorSample = new float[redMode.sampleSize()];

		//method to test the output of using the color sensor
		while(escape.isDown() == false) {

			redMode.fetchSample(colorSample, 0);
			System.out.println(redMode.fetchSample(colorSample, 0));

		}

		public static void main(String[] args) {

			LineFollowingAttempt1 testThing = new LineFollowingAttempt1();


		}

	}
	
}