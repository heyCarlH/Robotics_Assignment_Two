import java.util.LinkedList;
import java.util.Queue;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class Lab2 {

public static void main(String[] args) {
// set up ultrasensor
        EV3UltrasonicSensor ultrasensor = new EV3UltrasonicSensor(SensorPort.S2);
        SensorMode sonic = (SensorMode) ultrasensor.getDistanceMode();
        System.out.println("Press any key to start");
        Button.waitForAnyPress();
        
// create two motor objects to control the motors.
        EV3MediumRegulatedMotor rightWheel = new EV3MediumRegulatedMotor(MotorPort.A);
        EV3MediumRegulatedMotor leftWheel = new EV3MediumRegulatedMotor(MotorPort.B);
        
        // rotate right to take initial measurement
rightWheel.startSynchronization();
        rightWheel.rotateTo(410);
        leftWheel.rotateTo(410);
        rightWheel.endSynchronization();
        
        rightWheel.stop(true);
        leftWheel.stop(true);
                
        float[] sample_sonic = new float[sonic.sampleSize()];
        float initialDistance = 0;
        for (int i = 0; i < 3; i++) {
        sonic.fetchSample(sample_sonic, 0);
        initialDistance += sample_sonic[0];
        }
        
        // rotate back, head towards the wall, and stop at 0.15m before the wall
        rightWheel.startSynchronization();
        rightWheel.rotateTo(0);
        leftWheel.rotateTo(0);
        rightWheel.endSynchronization();
        
        moveForward(rightWheel,leftWheel,(initialDistance)/3-0.15);
        
        // rotate right and start following the wall
        rightWheel.startSynchronization();
        rightWheel.rotate(400);
        leftWheel.rotate(400);
        rightWheel.endSynchronization();
        
        // start to move slower
        leftWheel.setSpeed(200);
rightWheel.setSpeed(200);
        
        rightWheel.startSynchronization();       
        rightWheel.forward();
        leftWheel.forward();        
        rightWheel.endSynchronization();
        
        // take moving average of the sensor
        Queue<Float> lastThreeSamples = new LinkedList<>();
        float lastThreeSum = 0;
        for (int i = 0; i < 3; i++) {
        sonic.fetchSample(sample_sonic, 0);
            lastThreeSamples.add(sample_sonic[0]);
            lastThreeSum += sample_sonic[0];
        }
        float movingAverage = (float)lastThreeSum/3;
        System.out.println(movingAverage);
        while (movingAverage < 0.7f) {
        if (movingAverage < 0.05f) {
        //turn right a lot
        leftWheel.setSpeed(270);
        rightWheel.setSpeed(200);
        } else if (movingAverage < 0.1f) {
        //turn right a little
        leftWheel.setSpeed(250);
        rightWheel.setSpeed(200);
        } else if (movingAverage < 0.2f) {
        //go straight
        leftWheel.setSpeed(200);
        rightWheel.setSpeed(200);
        } else if (movingAverage < 0.25f){
        //turn left a little
        leftWheel.setSpeed(200);
        rightWheel.setSpeed(250);
        } else if (movingAverage < 0.7f){
        //turn left a lot
        leftWheel.setSpeed(200);
        rightWheel.setSpeed(270);
        }
         
        //take new measurement
        lastThreeSum -= lastThreeSamples.poll();
    sonic.fetchSample(sample_sonic, 0);
    int count = 0;
    while (sample_sonic[0] > 1.6f && count < 20) {
    sonic.fetchSample(sample_sonic, 0);
    count++;
    }
    lastThreeSamples.add(sample_sonic[0]);
    lastThreeSum += sample_sonic[0];
    movingAverage = lastThreeSum/3;
        }
        
        //the wall ends
        rightWheel.stop(true);
        leftWheel.stop(true);
        
        int leftSpeed = leftWheel.getSpeed();
        int rightSpeed = rightWheel.getSpeed();
        
        System.out.println("left speed: "+leftSpeed);
        System.out.println("right speed: "+rightSpeed);
        
        rightWheel.setSpeed(200);
        leftWheel.setSpeed(200);
        
        if (leftSpeed == 270) {
        rightWheel.startSynchronization();
            rightWheel.rotate(20);
            leftWheel.rotate(20);
            rightWheel.endSynchronization();
        } else if (leftSpeed == 250) {
        rightWheel.startSynchronization();
            rightWheel.rotate(10);
            leftWheel.rotate(10);
            rightWheel.endSynchronization();
        } /*else if (rightSpeed == 250 || rightSpeed == 270) {
        rightWheel.startSynchronization();
            rightWheel.rotate(-10);
            leftWheel.rotate(-10);
            rightWheel.endSynchronization();
        } */
        
        System.out.println(movingAverage); 
        moveForward(rightWheel,leftWheel,0.3);
        
        rightWheel.stop(true);
        leftWheel.stop(true);
        
        //turn left and start heading towards the goal
        rightWheel.startSynchronization();
        rightWheel.rotate(-390);
        leftWheel.rotate(-390);
        rightWheel.endSynchronization();
        
        rightWheel.stop(true);
        leftWheel.stop(true);
       
        rightWheel.startSynchronization();
        rightWheel.forward();
        leftWheel.forward();
        rightWheel.endSynchronization();
        
        Delay.msDelay(10200);
        
        rightWheel.stop();
        leftWheel.stop();
        
        rightWheel.close();
        leftWheel.close();
}
//@param: distance is in meters
private static void moveForward(EV3MediumRegulatedMotor motorA, EV3MediumRegulatedMotor motorB, double distance){
//motorA.synchronizeWith(new EV3MediumRegulatedMotor[] {motorB});
motorA.startSynchronization();       
        motorA.forward();
        motorB.forward();        
        motorA.endSynchronization();
        
        long time = (long) ((distance/0.175)*1000);
        Delay.msDelay(time);
        System.out.println(time);
        
        motorA.stop(true);
        motorB.stop(true);
}

}
