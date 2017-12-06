package integrated_code_4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

　
public class IntegratedCode4 {

	//Initialising error, integral and derivative constants
	//These will need to be adjusted appropriately
	public static final double kp = 285;
	public static final double ki = 0.00005;
	public static final double kd = 20;

	//Upon testing the sensor on the actual track, 0.45 was the value the observed value of having the sensor detecting exactly half of the line.
	public static final double target = 0.45;
	private double error = 0, integral = 0, last_error = 0, derivative;
	private int steeringValue;
	final double eq = 0.075;
	final double p = 200;
	final double d = 25;
	private double e = 0, der = 0, last_e = 0;
	double obstTurnVal = 0;

	//Adjust as necessary
	RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
	RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D); //PORT C IS BROKE AS FUCK
    private RegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(MotorPort.B);

    private EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S3);
    private SampleProvider ultraMode = ultraSensor.getDistanceMode();
    private float[] ultrasonicSample = new float[ultraMode.sampleSize()];

	//configure the colorSensor and it's redMode
   	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	SampleProvider redMode = colorSensor.getRedMode();
	float[] colorSample = new float[redMode.sampleSize()];
	

	public static void main(String[] args) {

		IntegratedCode4 test = new IntegratedCode4();
		test.move(175, 125);

	}

	public void move(int baseSpeed, int obstBaseSpeed) {

		//moveForward(leftMotor, rightMotor);
		
		redMode.fetchSample(colorSample, 0);
		rightMotor.setSpeed(20);
		leftMotor.setSpeed(20);
		rightMotor.forward();
		leftMotor.forward();
    	ultraMode.fetchSample(ultrasonicSample, 0);

		/*while(colorSample[0] < 0.8 && colorSample[0] > 0.67) {
			
			rightMotor.forward();
			leftMotor.forward();
			
		}*/
		
		while(true) {
			
			leftMotor.setSpeed(100); rightMotor.setSpeed(100);
			leftMotor.forward(); rightMotor.forward();
		
			while(ultrasonicSample[0] > 0.075) {
				
				redMode.fetchSample(colorSample, 0);
				ultraMode.fetchSample(ultrasonicSample, 0);
				//System.out.println(colorSample[0]);
				//implement PID control in order to get the appropriate steering value
				error = (double) target - colorSample[0];
				integral += error;	
				derivative = error - last_error;
				steeringValue = (int)((error * kp) + (integral * ki) + (derivative * kd));
			
				if(baseSpeed + steeringValue > 300) {
				
					leftMotor.setSpeed(300);
				
				} else {
				
					leftMotor.setSpeed(baseSpeed + steeringValue);
					//System.out.println("leftSpeed: " + leftMotor.getSpeed());
				
				}
			
				if(baseSpeed - steeringValue > 300) {
				
					rightMotor.setSpeed(300);
				
				} else {
				
					rightMotor.setSpeed(baseSpeed - steeringValue);
					//System.out.println("rightSpeed: " + rightMotor.getSpeed());
				
				}
			
			last_error = error;
			
			}
			
			rightMotor.stop(); leftMotor.stop();
			rightMotor.rotate(-130);
			leftMotor.rotate(130);
			sensorMotor.rotate(-85);
			obstTurnVal = 0;
			//rightMotor.setSpeed((int)(obstBaseSpeed + obstTurnVal));
			//leftMotor.setSpeed((int)(obstBaseSpeed + obstTurnVal));
			rightMotor.forward(); leftMotor.forward();

			while(colorSample[0] > 0.2) {
			
				ultraMode.fetchSample(ultrasonicSample, 0);
				redMode.fetchSample(colorSample, 0);
				e = eq - ultrasonicSample[0];
				der = e + last_e;
				obstTurnVal = (int)((p * e) + (d * der));
				System.out.println(ultrasonicSample[0]);
				
				if(ultrasonicSample[0] < 0.2) {
				
					if (obstBaseSpeed - obstTurnVal > 300) {rightMotor.setSpeed(300);}
					else {rightMotor.setSpeed((int)(obstBaseSpeed - obstTurnVal));}
					if(obstBaseSpeed + obstTurnVal > 300) {leftMotor.setSpeed(300);}
					else {leftMotor.setSpeed((int)(obstBaseSpeed + obstTurnVal));}
					
				} else {rightMotor.setSpeed(100); leftMotor.setSpeed(50);}
				
				last_e = e;
				
			}
			
			rightMotor.stop(); leftMotor.stop();
			sensorMotor.rotate(85);
			Delay.msDelay(1000);
			ultraMode.fetchSample(ultrasonicSample, 0);
			rightMotor.rotate(-120);
			leftMotor.rotate(150);
			
			
			
		}

　
	}

}
