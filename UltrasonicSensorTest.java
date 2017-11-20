public class UltrasonicSensorTest {
	
	
    private RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
    private RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D); //PORT C IS BROKE AS FUCK
    private RegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(MotorPort.B);

    private EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S3);
    private SampleProvider ultraMode = ultraSensor.getDistanceMode();
    private float[] ultrasonicSample = new float[ultraMode.sampleSize()];

    private int sensorAngle = 0;
    //if we reach an obstacle less than 20 cm away, begin turning right
    float lastReading = ultraMode.fetchSample(ultrasonicSample, 0);


    while(lastReading > 0.15 && lastReading <  0.2){ //logic for when the sensor first finds an obstacle
        rightMotor.rotate(60);
        sensorMotor.rotate(60);
        sensorAngle = + sensorAngle + 60;
        rightMotor.rotate(60); //trying to move it forward
        leftMotor.rotate(60);
        lastReading = ultraMode.fetchSample(ultrasonicSample, 0);
    }
    
    double increasing = 1.0; //just a read for if the distance from the sensor is increasing or not
    
    while(lR2 > 0;){ //or until a black line is found
        leftMotor.rotate(60); //the sensor should still be rotating in the same direction
        sensorAngle = sensorAngle + 60;
        rightMotor.rotate(60); //trying to move it forward
        leftMotor.rotate(60);
        double lR2 = ultraMode.fetchSample(ultrasonicSample, 0);
        increasing = lr2 - lastReading;
        lastReading = lr2;
    }
    //^ we keep track of the angle of the sensor so we can
    // 1. correct it when the obstacle has passed
    // 2. have some way of keeping track how off course the robot is from the original track

    //turn the sensor back to facing the front using the sensor angle
    
    
	//just see what might be used for the ultrasonic range
	/*
	 * set up new differential pilot(not sure)
	 * set up brick
	 * Brick b = BrickFinder.getDefault();
	 * set up sensor port
	 * create new EV3UltrasonicSensor (already mentioned)
	 * new Ultrasonic(getMode("Distance"))
	 */
    
    //another way we probably can do, maybe we can create an ultrasonic distance class 
    //should we stop the robot first once finds the obstacle, call the float data pulled in
    //check the distance, if the distance is less than () cm, robot stops, else if...
}
