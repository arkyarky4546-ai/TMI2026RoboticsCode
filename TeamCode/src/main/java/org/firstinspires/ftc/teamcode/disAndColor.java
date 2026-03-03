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
    private volatile double currentPos = 0;

    public disAndColor(servo720Rot servRot) {
        this.servRot = servRot;
        //this.disBL = disBL;
        //this.disST = disST;
    }

    @Override //ts is good practice to comment that you are overriding the thread class stuff. You need to in order to run stuff and you need to call (variable that is a thread).start() to start running the thread
    public void run() {
        while (running) {
            if (servRot != null && servRot.getColors() == 1) {
                greenPos = currentPos;
            }
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

   // public boolean hasBallBL() { return blHasBall; }
    //public boolean hasBallST() { return stHasBall; }
    public double foundGreen() {return greenPos; }

    public void stopThread(){
        running = false;
    }
    public void upColor(double position){
        currentPos = position;
    }
}