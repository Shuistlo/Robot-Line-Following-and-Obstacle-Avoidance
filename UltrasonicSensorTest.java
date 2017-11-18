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
        rightMotor.turn(60);
        sensorMotor.turn(60);
        sensorAngle = + sensorAngle + 60;
        //move robot forward a few cm
        lastReading = ultraMode.fetchSample(ultrasonicSample, 0);
    }
    while(lastReading is increasing){ //or until a black line is found
        rightMotor.turn(60);
        sensorAngle = + sensorAngle + 60;
        //move robot forward a few cm
        lastReading = ultraMode.fetchSample(ultrasonicSample, 0);
    }
    //^ we keep track of the angle of the sensor so we can
    // 1. correct it when the obstacle has passed
    // 2. have some way of keeping track how off course the robot is from the original track

    //turn the sensor back to facing the front using the sensor angle
}
