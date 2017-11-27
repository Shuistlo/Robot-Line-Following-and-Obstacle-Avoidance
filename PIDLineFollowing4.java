package line_following_attempt4;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;           
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.UnregulatedMotor;
import java.lang.Math;

public class PIDLineFollowing4 {

	//Initialising error, integral and derivative constants
	//These will need to be adjusted appropriately
	public static final double kp = 250;
	public static final double ki = 1;
	public static final double kd = 7;

	//Upon testing the sensor on the actual track, 0.45 was the value the observed value of having the sensor detecting exactly half of the line.
	public static final double target = 0.45;

	private double error = 0, integral = 0, last_error = 0, derivative;
	private int steeringValue;

	//Adjust as necessary
	RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
	RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D); //PORT C IS BROKE AS FUCK

	//configure the colorSensor and it's redMode
   	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	SampleProvider redMode = colorSensor.getRedMode();
	float[] colorSample = new float[redMode.sampleSize()];
	

	public static void main(String[] args) {

		PIDLineFollowing4 test = new PIDLineFollowing4();
		test.move(175);

	}

	public void move(int baseSpeed) {

		//moveForward(leftMotor, rightMotor);
		
		redMode.fetchSample(colorSample, 0);
		rightMotor.setSpeed(20);
		leftMotor.setSpeed(20);
		rightMotor.forward();
		leftMotor.forward();

		/*while(colorSample[0] < 0.8 && colorSample[0] > 0.67) {
			
			rightMotor.forward();
			leftMotor.forward();
			
		}*/

		while(true) {
			
			redMode.fetchSample(colorSample, 0);
			System.out.println(colorSample[0]);
			//implement PID control in order to get the appropriate steering value
			error = (double) target - colorSample[0];
			integral += error;	
			derivative = error - last_error;
			steeringValue = (int)((error * kp) + (integral * ki) + (derivative * kd));
			
			if(baseSpeed + steeringValue > 300) {
				
				//leftMotor.setSpeed(300);
				leftMotor.setSpeed(baseSpeed + steeringValue);
				
			} else {
				
				leftMotor.setSpeed(baseSpeed + steeringValue);
				System.out.println("leftSpeed: " + leftMotor.getSpeed());
				
			}
			
			if(baseSpeed - steeringValue > 300) {
				
				rightMotor.setSpeed(baseSpeed - steeringValue);
				//rightMotor.setSpeed(300);
				
			} else {
				
				rightMotor.setSpeed(baseSpeed - steeringValue);
				System.out.println("rightSpeed: " + rightMotor.getSpeed());
				
			}
			
			
			
			/*leftMotor.setSpeed(150 + steeringValue);
			//System.out.println("leftSpeed: " + leftMotor.getSpeed());
			rightMotor.setSpeed(150 - steeringValue);
			//System.out.println("rightSpeed: " + rightMotor.getSpeed());*/
			
			
			last_error = error;
			
		}

	}

}

