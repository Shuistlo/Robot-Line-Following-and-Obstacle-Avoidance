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


public class PIDLineFollowing {

	//Initialising error, integral and derivative constants
	public static final double kp = 1;
	public static final double ki = 1;
	public static final double kd = 1;

	//USE THE SENSOR ON THE ACTUAL TRACK TO FIND A GOOD VALUE FOR THIS
	public static final double target = 0;

	private double error = 0, integral = 0, last_error, derivative;

	//Adjust motorports as necessary
	RegulatedMotor leftMotor = new RegulatedMotor(MotorPort.A);
	RegulatedMotor rightMotor = new RegulatedMotor(MotorPort.B);

	//initialise motors colorSensor and set it to the correct port
	Brick brick = new BrickFinder.getDefault();
	Key escapeKey = brick.getKey("escpae");
	Port s1 = LocalEV3.get().getPort("S1");

	//configure the colorSensor and it's redMode
   	EV3ColorSensor colorSensor = new EV3ColorSensor(s1);
	SampleProvider redMode = colorSensor.getRedMode();
	float[] colorSample = new float[redMode.sampleSize()];

	public static void main(String[] args) {

		move();

	}

	public static void move() {

		while(true) {

			//implement PID control in order to get the appropriate steering value
			last_error = error;
			error = target - redMode.fetchSample(colorSample, 0);
			integral += error;					
			derivative = error - last_error;
			steeringValue = (error * kp) + (integral * ki) + (derivative * kd);

			//steer in the direction and magnitude of the steeringValue


		}



	}
	
	//might be needed at some point
	public void moveForward(RegulatedMotor motorA, RegulatedMotor motorB) {

		motorA.forward();
		motorB.forward();

	}

}
