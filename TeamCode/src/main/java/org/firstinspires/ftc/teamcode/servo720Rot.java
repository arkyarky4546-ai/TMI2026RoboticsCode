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
    public DistanceSensor[] distanceSensors = new DistanceSensor[6];
    private DistanceSensor mainDis;

    //double arrays
    private double[] positionHoldIntake = new double[7];
    private double[] positionHoldShoot = new double[7];

    //int arrays
    private int[] intakePosOpen = new int[] {0 , 0 , 0};
    private int[] shootPosOpen = new int[] {0 , 0 , 0};

    //ints
    int ballzCM = 6;
    //bools

    //doubles
    double distanceCM = 0.0;


    public servo720Rot(HardwareMap hardwareMap, String servoName, String servoName2, String distance1, String distance2, String distance3, String distance4, String distance5, String distance6, String distance7){

        //constructor to initialize everything
        servo = hardwareMap.get(Servo.class, servoName);
        servo2 = hardwareMap.get(Servo.class, servoName2);

        //main distance sensor and all the other distance sensors
        mainDis = hardwareMap.get(DistanceSensor.class, distance1);
        distanceSensors[0] = hardwareMap.get(DistanceSensor.class, distance2);
        distanceSensors[1] = hardwareMap.get(DistanceSensor.class, distance3);
        distanceSensors[2] = hardwareMap.get(DistanceSensor.class, distance4);
        distanceSensors[3] = hardwareMap.get(DistanceSensor.class, distance5);
        distanceSensors[4] = hardwareMap.get(DistanceSensor.class, distance6);
        distanceSensors[5] = hardwareMap.get(DistanceSensor.class, distance7);

        //these are the various shoot and intake positions (since the gear ratio is 5/2, I chose the positions in order to have 720 degrees of rotation)
        positionHoldShoot = new double[] {0.0740442655936 , 0.222132796781 , 0.370221327968 , 0.518309859155 , 0.666398390342 , 0.814486921529 , 0.962575452716};
        positionHoldIntake = new double[] {0 , 0.150234741784 , 0.300469483568 , 0.450704225352 , 0.600938967136 , 0.75117370892 , 0.901408450704};
    }
    public double getDisMain(){
        return mainDis.getDistance(DistanceUnit.CM);
    }
    //servoSetPosition
    //@twins, if you want to use this, make sure to specify if you are in shoot position or in intake position
    //offset = 1 means shoot posotion so this references posHoldShoot array. offset = 0 means intake position
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
    //gets current servo positions
    public double getPos(){
        return servo2.getPosition();
    }
    //this method just returns true if there is a ball and false if not, useful when checking which spots we have open.
    private boolean hasBall(int position) {
        return distanceSensors[position].getDistance(DistanceUnit.CM) < ballzCM;
    }
    //indices to check for for the array of distance sensors so we dont have to check all of them
    private int[] getIntakeIndices() {
        return new int[] {0, 2, 4};
    }

    private int[] getShootIndices() {
        return new int[] {1, 3, 5};
    }
    //this gets the nearest open position so you can easily rotate to it mode is either 1 or 0, 1 for shoot and 0 for intake
    public int getFree(int mode, double currentPos) {

        int currentIndex = 0;

        //checks to see which position indec we are in
        for(int i = 0; i < 7; i++){
            if(mode == 1){
                if(positionHoldShoot[i] == currentPos){
                    currentIndex = i;
                    break;
                }
            }
            else{
                if(positionHoldIntake[i] == currentPos){
                    currentIndex = i;
                    break;
                }
            }
        }
        int[] useInd;
        //from the mode that we are in either 1 for shoot or 0 for intake, we decide which distance sensors that we want to check
        if(mode == 0){
            useInd = getIntakeIndices();
        }
        else{
            useInd = getShootIndices();
        }
        //this boolean is to see whether or not we are looking to shoot a ball or intake a ball
        boolean lookingForBall;
        if(mode == 1){
            lookingForBall = true;
        }
        else{
            lookingForBall = false;
        }

        int close = 67676767;
        int minD = 67676767;
        //cycles through to check which ones has balls or doesnt have balls based on lookingForBall boolean
        for (int index : useInd) { //why am I the goat of java (definitely knew how to do this and didnt learn this on 1/18/2026
            if (hasBall(index) == lookingForBall) {
                int distance = Math.abs(index - currentIndex); //calculates the shortest path to the open position for intake and the position with the ball for shooting

                if (distance < minD) {
                    minD = distance;
                    close = index;
                }
            }
        }
        if(close > 6){ // makes sure not to return 6767676767 so if we cant find any then we just return the same index
            return currentIndex;
        }
        else {//idk if this logic works, I think it does though cause like 3rotate = 360 even for intake, and odd for shoot
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
    public void regRot (double Pos){
        int currentIndex = 0;
        for(int i = 0; i < 7; i++){
                if(positionHoldIntake[i] == Pos){
                    currentIndex = i;
                    break;
                }
        }
        currentIndex += 1;
        if(currentIndex == 7){
            currentIndex = 0;
        }
        sSP(currentIndex, 0);
    }
}

