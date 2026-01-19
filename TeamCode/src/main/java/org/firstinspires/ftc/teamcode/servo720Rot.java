package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class servo720Rot {
    //servos
    private Servo servo;
    private Servo servo2;

    //distance sensors
    private DistanceSensor[] distanceSensors = new DistanceSensor[6];
    private DistanceSensor mainDis;
    //double arrays
    private double[] positionHoldIntake = new double[7];
    private double[] positionHoldShoot = new double[7];

    //int arrays
    private int[] intakePosOpen = new int[] {0 , 0 , 0};
    private int[] shootPosOpen = new int[] {0 , 0 , 0};

    //ints
    int ballzCM = 3;
    //bools

    //doubles
    double distanceCM = 0.0;


    public servo720Rot(HardwareMap hardwareMap, String servoName, String servoName2, String distance1, String distance2, String distance3, String distance4, String distance5, String distance6, String distance7){
        servo = hardwareMap.get(Servo.class, servoName);
        servo2 = hardwareMap.get(Servo.class, servoName2);

        mainDis = hardwareMap.get(DistanceSensor.class, distance1);
        distanceSensors[0] = hardwareMap.get(DistanceSensor.class, distance2);
        distanceSensors[1] = hardwareMap.get(DistanceSensor.class, distance3);
        distanceSensors[2] = hardwareMap.get(DistanceSensor.class, distance4);
        distanceSensors[3] = hardwareMap.get(DistanceSensor.class, distance5);
        distanceSensors[4] = hardwareMap.get(DistanceSensor.class, distance6);
        distanceSensors[5] = hardwareMap.get(DistanceSensor.class, distance7);

        positionHoldShoot = new double[] {0.0500782472613 , 0.200312989045 , 0.350547730829 , 0.500782472613 , 0.651017214397 , 0.801251956182 , 0.951486697966};
        positionHoldIntake = new double[] {0 , 0.150234741784 , 0.300469483568 , 0.450704225352 , 0.600938967136 , 0.75117370892 , 0.901408450704};
    }
    public double getDisMain(){
        return mainDis.getDistance(DistanceUnit.CM);
    }
    //servoSetPosition
    //@twins, if you want to use this, make sure to specify if you are in shoot position or in intake position
    //offset = 1 means shoot psotion so this references posHoldShoot array. offset = 0 means intake position
    //int angle means what position you want the servo to go to range between 0 to 6
    public void sSP(int angle, int offset){
        if(offset == 0) { // intake Pos
            servo.setPosition(positionHoldIntake[angle]);
            servo2.setPosition(positionHoldIntake[angle]);
        }
        if(offset == 1){ // shoot Pos
            servo.setPosition(positionHoldShoot[angle]);
            servo2.setPosition(positionHoldShoot[angle]);
        }
    }

    //this method just returns true if there is a ball and false if not.
    private boolean hasBall(int position) {
        return distanceSensors[position].getDistance(DistanceUnit.CM) < ballzCM;
    }
    private int[] getIntakeIndices() {
        return new int[] {1, 3, 5};
    }

    private int[] getShootIndices() {
        return new int[] {0, 2, 4, 6};
    }
    //this gets the nearest open position so you can easily rotate to it mode is either 1 or 0, 1 for shoot and 0 for intake
    public int getFree(int currentIndex, int mode, double currentPos) {
        int[] useInd;
        if(mode == 0){
            useInd = getIntakeIndices();
        }
        else{
            useInd = getShootIndices();
        }
        boolean lookingForBall; //maybe not efficient declaring it everytime, but its a primitive type soooooo
        if(mode == 1){
            lookingForBall = true;
        }
        else{
            lookingForBall = false;
        }

        int close = 67676767;
        int minD = 67676767;

        for (int index : useInd) { //why am I the goat of java (definitely knew how to do this and didnt learn this on 1/18/2026
            if (hasBall(index) == lookingForBall) {
                int distance = Math.abs(index - currentIndex);

                if (distance < minD) {
                    minD = distance;
                    close = index;
                }
            }
        }
        if(close > 6){
            return currentIndex;
        }
        else {//idk if this logic works, I think it does though cause like 3rotate = 360
            if(close % 2 == 0){
                close = close/2;
                if((Math.abs(positionHoldIntake[close] - positionHoldIntake[currentIndex]) < Math.abs(positionHoldIntake[close + 3] - - positionHoldIntake[currentIndex]))){
                    return close;
                }
                else{
                    return close + 3;
                }
            }
            if(close % 2 == 1){
                close = close - 1;
                close = close/2;
                if((Math.abs(positionHoldShoot[close] - positionHoldShoot[currentIndex]) < Math.abs(positionHoldShoot[close + 3] - - positionHoldShoot[currentIndex]))){
                    return close;
                }
                else{
                    return close + 3;
                }
            }

            return close;
        }
    }
}
