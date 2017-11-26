package line_following_attempt5;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import java.lang.Math;
import lejos.nxt.NXTMotor;

public class PIDLineFollowing5 {

	//Initialising error, integral and derivative constants
	//These will need to be adjusted appropriately
	public static final double kp = 1.2;
	public static final double ki =  0.0008;
	public static final double kd = 5;

	//Upon testing the sensor on the actual track, 0.45 was the value the observed value of having the sensor detecting exactly half of the line.
	public static final double target = 0.45;

	private double error = 0, integral = 0, last_error = 0, derivative;
	private int steeringValue;

	//Adjust motorports as necessary
	RegulatedMotor leftMotor = new RegulatedMotor(MotorPort.A);
	RegulatedMotor rightMotor = new RegulatedMotor(MotorPort.D); //PORT C IS BROKE AS FUCK

	//configure the colorSensor and it's redMode
   	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	SampleProvider redMode = colorSensor.getRedMode();
	float[] colorSample = new float[redMode.sampleSize()];
	

	public static void main(String[] args) {

		PIDLineFollowing5 test = new PIDLineFollowing5();
		test.move();

	}

	public void move() {

		//moveForward(leftMotor, rightMotor);
		
		redMode.fetchSample(colorSample, 0);
		rightMotor.setSpeed(20);
		leftMotor.setSpeed(20);
		rightMotor.forward();
		leftMotor.forward();
		

		while(true) {
			
			redMode.fetchSample(colorSample, 0);
			//implement PID control in order to get the appropriate steering value
			error = (double) target - colorSample[0];
			integral += error;	
			derivative = error - last_error;
			steeringValue = (int)((error * kp) + (integral * ki) + (derivative * kd));
			
			leftMotor.setSpeed(20 - steeringValue);
			rightMotor.setSpeed(20 + steeringValue);


			last_error = error;
			
		}


　
	}

}

