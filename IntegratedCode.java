
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class IntegratedCode {

	//Initialising error, integral and derivative constants
	//These will need to be adjusted appropriately
	public static final double kp = 285;
	public static final double ki = 1.5;
	public static final double kd = 15;

	//Upon testing the sensor on the actual track, 0.45 was the value the observed value of having the sensor detecting exactly half of the line.
	public static final double target = 0.45;
	private double error = 0, integral = 0, last_error = 0, derivative;
	private int steeringValue;

	//Adjust as necessary
	RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
	RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D); //PORT C IS BROKE AS FUCK
    private RegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(MotorPort.B);

    private EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);
    private SampleProvider ultraMode = ultraSensor.getDistanceMode();
    private float[] ultrasonicSample = new float[ultraMode.sampleSize()];

	//configure the colorSensor and it's redMode
   	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	SampleProvider redMode = colorSensor.getRedMode();
	float[] colorSample = new float[redMode.sampleSize()];
	

	public static void main(String[] args) {

		IntegratedCode test = new IntegratedCode();
		test.move();

	}

	public void move() {

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
		
		while(ultrasonicSample[0] > 0.2) {
			
			redMode.fetchSample(colorSample, 0);
			ultraMode.fetchSample(ultrasonicSample, 0);
			System.out.println(colorSample[0]);
			//implement PID control in order to get the appropriate steering value
			error = (double) target - colorSample[0];
			integral += error;	
			derivative = error - last_error;
			steeringValue = (int)((error * kp) + (integral * ki) + (derivative * kd));
			
			if(150 + steeringValue > 300) {
				
				leftMotor.setSpeed(300);
				
			} else {
				
				leftMotor.setSpeed(150 + steeringValue);
				System.out.println("leftSpeed: " + leftMotor.getSpeed());
				
			}
			
			if(150 - steeringValue > 300) {
				
				rightMotor.setSpeed(300);
				
			} else {
				
				rightMotor.setSpeed(150 - steeringValue);
				System.out.println("rightSpeed: " + rightMotor.getSpeed());
				
			}
			
			
			
			/*leftMotor.setSpeed(150 + steeringValue);
			//System.out.println("leftSpeed: " + leftMotor.getSpeed());
			rightMotor.setSpeed(150 - steeringValue);
			//System.out.println("rightSpeed: " + rightMotor.getSpeed());*/
			
			
			last_error = error;
			
		}
		while(ultrasonicSample[0] < 0.2){
			//make the robot rotate 45 degrees clockwise in place
			rightMotor.rotate(-45);
			leftMotor.rotate(45);
	
			//perform synchronised movement forwards:
			moveForwardABit(360);
	
			//turn sensor until it detects the object again
			sensorMotor.rotate(-90);
			ultraMode.fetchSample(ultrasonicSample, 0);
		}
		while(ultrasonicSample[0] < 0.2) {
			
			while(colorSample[0] < 0.2) {
								
				redMode.fetchSample(colorSample, 0);
				moveForwardABit(360);

				while(ultrasonicSample[0] > 0.3) {
				
					ultraMode.fetchSample(ultrasonicSample, 0);
					sensorMotor.rotate(-10);
				
				}

			}
			
		}

	}


	}
	
	private void moveForwardABit(int distance) {
		
		leftMotor.startSynchronization();
		leftMotor.rotate(distance, true);
		rightMotor.rotate(distance, true);
		leftMotor.endSynchronization();
		leftMotor.waitComplete();
		rightMotor.waitComplete();
		
	}

}
