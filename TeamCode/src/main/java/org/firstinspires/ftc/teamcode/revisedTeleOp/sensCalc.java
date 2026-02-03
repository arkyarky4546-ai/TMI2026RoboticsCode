package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.servo720Rot;

public class sensCalc extends Thread {
    private volatile boolean running = true;
    private servo720Rot servRot;

    private volatile double intakeDis = 0; // need this as volitile as I am getting this in the main thread
    private ElapsedTime sensorTimer = new ElapsedTime();
    private final int updateTime = 20;
    public sensCalc(servo720Rot servRot) {
        this.servRot = servRot;
    }

    @Override //ts is good practice to comment that you are overriding the thread class stuff. You need to in order to run stuff and you need to call (variable that is a thread).start() to start running the thread
    public void run(){
        while (running){
                if (sensorTimer.milliseconds() > updateTime){
                    intakeDis = servRot.getDis23();
                    sensorTimer.reset();
                }
            try {
                Thread.sleep(10); //something called cpu spinning
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    public double getIntakeDistance(){ //this is why we need the volitile (writes to a cpu cahce, but not updated)
        return intakeDis;
    }

    public void stopThread(){
        running = false;
    }
}