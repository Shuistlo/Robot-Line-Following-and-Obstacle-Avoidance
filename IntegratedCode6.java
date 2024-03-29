package integrated_code_6;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

　
public class IntegratedCode6 {

	//Initialising error, integral and derivative constants
	//These will need to be adjusted appropriately
	public static final double kp = 285;
	public static final double ki = 0.00005;
	public static final double kd = 20;

	//Upon testing the sensor on the actual track, 0.45 was the value the observed value of having the sensor detecting exactly half of the line.
	public static final double target = 0.45;
	private double error = 0, integral = 0, last_error = 0, derivative;
	private int steeringValue;
	final double eq = 0.055;
	final double p = 250;
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

		IntegratedCode6 test = new IntegratedCode6();
		test.move(175, 200);

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
		
			while(ultrasonicSample[0] > 0.1) {
				
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
			sensorMotor.rotate(-90);
			obstTurnVal = 0;
			rightMotor.forward(); leftMotor.forward();

			while(colorSample[0] > 0.12) {
			
				ultraMode.fetchSample(ultrasonicSample, 0);
				redMode.fetchSample(colorSample, 0);
				//System.out.println(colorSample[0]);
				e = eq - ultrasonicSample[0];
				der = e + last_e;
				obstTurnVal = (int)((p * e) + (d * der));
				System.out.println(colorSample[0]);
				
				if(ultrasonicSample[0] < 0.2) {
				
					if (obstBaseSpeed - obstTurnVal > 300) {rightMotor.setSpeed(300);}
					else {rightMotor.setSpeed((int)(obstBaseSpeed - obstTurnVal));}
					if(obstBaseSpeed + obstTurnVal > 300) {leftMotor.setSpeed(300);}
					else {leftMotor.setSpeed((int)(obstBaseSpeed + obstTurnVal));}
					
				} else {rightMotor.setSpeed(150); leftMotor.setSpeed(75);}
				
				last_e = e;
				
			}
			
			rightMotor.stop(); leftMotor.stop();
			sensorMotor.rotate(90);
			Delay.msDelay(1000);
			ultraMode.fetchSample(ultrasonicSample, 0);
			redMode.fetchSample(colorSample, 0);
			rightMotor.setSpeed(135); leftMotor.setSpeed(100);
			/*rightMotor.rotate(-100);
			leftMotor.rotate(180);
			
			//System.out.println(colorSample[0]);
			if(colorSample[0] > 0.3) {turn();}
			//moveForwardABit(120);*/
			rightMotor.rotate(-170);
			redMode.fetchSample(colorSample, 0);
			while(colorSample[0] > 0.1) {
				
				redMode.fetchSample(colorSample, 0);
				leftMotor.forward();
				rightMotor.forward();
				
			}
			
			rightMotor.stop(); leftMotor.stop();
			moveBackABit(-100);
			rightMotor.rotate(-75);
			redMode.fetchSample(colorSample, 0);
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(75);
			redMode.fetchSample(colorSample, 0);
			while(colorSample[0] > 0.1) {
				
				redMode.fetchSample(colorSample, 0);
				leftMotor.forward();
				rightMotor.forward();
				
			}
			
			rightMotor.stop(); leftMotor.stop();
			moveBackABit(-100);
			rightMotor.rotate(-75);
			redMode.fetchSample(colorSample, 0);
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(85);
			redMode.fetchSample(colorSample, 0);
			while(colorSample[0] > 0.1) {
				
				redMode.fetchSample(colorSample, 0);
				leftMotor.forward();
				rightMotor.forward();
				
			}
						
		}

　
	}
  
  private void moveBackABit(int distance) {
		
	  	leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.startSynchronization();
		leftMotor.rotate(distance, true);
		rightMotor.rotate(distance, true);
		leftMotor.endSynchronization();
		leftMotor.waitComplete();		
		rightMotor.waitComplete();
		
	}
	
	private void spin() {
		
		redMode.fetchSample(colorSample, 0);
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(200);

		while(colorSample[0] > 0.075) {
			
			redMode.fetchSample(colorSample, 0);
			rightMotor.forward();
			
		}
		
	}

}
