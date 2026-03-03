package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.servo720Rot;

public class disAndColor extends Thread {
    private volatile boolean running = true;
    private servo720Rot servRot;

    private volatile double intakeDis = 0; // need this as volitile as I am getting this in the main thread
    private ElapsedTime sensorTimer = new ElapsedTime();
    private final int updateTime = 10;

    private DistanceSensor disBL;
    private DistanceSensor disST;

    // volatile ensures the main thread always reads the most recent value from memory
    private volatile boolean blHasBall = false;
    private volatile boolean stHasBall = false;
    private volatile double greenPos = 0.0;

    public disAndColor(servo720Rot servRot, DistanceSensor disBL, DistanceSensor disST) {
        this.servRot = servRot;
        this.disBL = disBL;
        this.disST = disST;
    }

    @Override //ts is good practice to comment that you are overriding the thread class stuff. You need to in order to run stuff and you need to call (variable that is a thread).start() to start running the thread
    public void run(){
        while (running){
            if(servRot.getColors() == 1){
                greenPos = servRot.getPos();
            }
            if (disBL != null) {
                blHasBall = disBL.getDistance(DistanceUnit.CM) < 4.0;
            }
            if (disST != null) {
                stHasBall = disST.getDistance(DistanceUnit.CM) < 4.0;
            }
            try {
                Thread.sleep(20); //something called cpu spinning
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    public boolean hasBallBL() { return blHasBall; }
    public boolean hasBallST() { return stHasBall; }
    public double foundGreen() {return greenPos; }

    public void stopThread(){
        running = false;
    }
}