import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/*
 * This will help me to determint which values are returned for black, and which are returned for white. This should allow me to write a program that
 * will make the robot follow a straight black line. Turning implementation will be added in later.
 */

public class LineFollowingAttempt1 {

	public LineFollowingAttempt1() {
		//motorA is the small motor at the top which we will not use for now
		EV3LargeRegulatedMotor motorB = new EV3LargeRegulatedMotor(MotorPort.B);
		EV3LargeRegulatedMotor motorC = new EV3LargeRegulatedMotor(MotorPort.C);
		
		//declaring the Brick and brickfinder may not be necessary as the brick has already been connected in the preferences
		//up to you tbh
		new BrickFinder();
		Brick brick = BrickFinder.getDefault();
		
		/*
		Key escapeKey = brick.getKey("escape"); //this is not necessary
		Port s1 = LocalEV3.get().getPort("S1");

		//configure the colorSensor and it's redMode
    	EV3ColorSensor colorSensor = new EV3ColorSensor(s1);
    	*/
		
        Port s2 = brick.getPort("S2");
        EV3ColorSensor sensor = new EV3ColorSensor(SensorPort.S2);
        SensorMode red = sensor.getRedMode();
        float[] colorSample = new float[red.sampleSize()];
        
        /*
         * This while loop prints about 0-0.2 for black color and generally above 0.7 for a white surface
         */
        /*while(true) {
        	
        	red.fetchSample(colorSample, 0);
        	System.out.println(colorSample[0]);
        	
        }*/
        
        while(true) {
        	
        	red.fetchSample(colorSample, 0);
        	
        	while(colorSample[0] < 0.3) {
        		
        		red.fetchSample(colorSample, 0);
        		
        		motorB.forward();
        		motorC.forward();
        		
        		
        	}
        	
        	while(colorSample[0] > 0.3) {
        		
        		red.fetchSample(colorSample, 0);
        		
        		motorB.rotate(60); 
            		motorC.rotate(-60);
        		
        	}
        	
        }
        
        
        //UNCOMMENT THIS
       /*while(sensor.getColorID() == 1){ //colour ID 0-7, 1 is black
        	//going to need to use multithreading to run these two methods at the same time 
        	motorB.rotate(60); 
        	motorC.rotate(-60);
        } 
        
        /*
		SampleProvider redMode = colorSensor.getRedMode();
		float[] colorSample = new float[redMode.sampleSize()];

		//method to test the output of using the color sensor
		while(escapeKey.isDown() == false) {

			redMode.fetchSample(colorSample, 0);
			System.out.println(redMode.fetchSample(colorSample, 0));

		}
		*/

	}
	public static void main(String[] args) {

		LineFollowingAttempt1 testThing = new LineFollowingAttempt1();

　
	}
	
}
