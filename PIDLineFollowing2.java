package line_following_attempt2;

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

public class PIDLineFollowing2 {

	//Initialising error, integral and derivative constants
	//These will need to be adjusted appropriately
	public static final double kp = 1;
	public static final double ki =  0;
	public static final double kd = 0;

	//Upon testing the sensor on the actual track, 0.45 was the value the observed value of having the sensor detecting exactly half of the line.
	public static final double target = 0.45;

	private double error = 0, integral = 0, last_error = 0, derivative;
	private int steeringValue, leftSpeed, rightSpeed;
	private boolean leftForward, rightForward;

	//Adjust motorports as necessary
	RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
	RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D); //PORT C IS BROKE AS FUCK

	//configure the colorSensor and it's redMode
   	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	SampleProvider redMode = colorSensor.getRedMode();
	float[] colorSample = new float[redMode.sampleSize()];
	
	//initialise ultrasound sensor
	EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S3);
	SampleProvider ultraMode = ultraSensor.getDistanceMode();

	public static void main(String[] args) {

		PIDLineFollowing2 test = new PIDLineFollowing2();
		test.move();

	}

	public void move() {

		//moveForward(leftMotor, rightMotor);
		
		redMode.fetchSample(colorSample, 0);
		rightMotor.setSpeed(180);
		leftMotor.setSpeed(180);
		rightMotor.forward();
		rightForward = true;
		leftMotor.forward();
		leftForward = true;
		

		while(true) {
			
			redMode.fetchSample(colorSample, 0);
			//implement PID control in order to get the appropriate steering value
			error = (double) target - colorSample[0];
			integral += error;	
			derivative = error - last_error;
			steeringValue = (int)((error * kp) + (integral * ki) + (derivative * kd));
			leftSpeed = leftMotor.getSpeed();
			rightSpeed = rightMotor.getSpeed();


			if(steeringValue < 0) {

				if(leftForward == true) {

					leftMotor.setSpeed(leftSpeed - steeringValue);

				} else {

					if(leftSpeed + steeringValue < 0) {

						leftMotor.forward(Math.abs(leftSpeed + steeringValue));
						leftForward = true;

					} else {

						leftMotor.setSpeed(leftSpeed + steeringValue);

					}

				}

				if(rightForward == false) {

					rightMotor.setSpeed(rightSpeed - steeringValue);

				} else {

					if(rightSpeed + steeringValue < 0) {

						rightMotor.backward(Math.abs(rightSpeed - steeringValue));
						rightForward = false;

					} else {

						rightMotor.setSpeed(rightSpeed + steeringValue);

					}

				}

			} else {

				if(rightForward == true) {

					rightMotor.setSpeed(rightSpeed + steeringValue);

				} else {

					if(rightSpeed - steeringValue < 0) {

						rightMotor.forward(steeringValue - rightSpeed);
						rightForward = true;

					} else {

						rightMotor.setSpeed(rightSpeed - steeringValue);

					}

				}

				if(leftForward == false) {

					leftMotor.setSpeed(leftSpeed + steeringValue);

				} else {

					if(leftSpeed - steeringValue < 0) {

						leftMotor.backward(steeringValue - leftSpeed);
						leftForward = false;

					} else {

						leftMotor.setSpeed(leftSpeed - steeringValue);

					}

				}

			}
			
			last_error = error;
			
		}

　
　
	}
	
	//might be needed at some point
	public void moveForward(int steeringValue) {

		leftMotor.setSpeed(90 + steeringValue);
		rightMotor.setSpeed(90 - steeringValue);
		leftMotor.forward();
		rightMotor.forward();

	}

}














if(leftMotor.getSpeed() > 0) {

				if(leftMotor.getSpeed() - steeringValue > 0) {

					leftMotor.setSpeed(leftMotor.getSpeed() - steeringValue);

				} else {

					leftMotor.backward(steeringValue - leftMotor.getSpeed());

				}

			} else {

				if(rightMotor.getSpeed() )

			}
