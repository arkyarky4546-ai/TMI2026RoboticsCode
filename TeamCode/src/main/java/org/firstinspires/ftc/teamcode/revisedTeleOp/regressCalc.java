package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

public class regressCalc extends Thread {
    private volatile boolean running = true;
    private volatile double targetVel = 0.0;
    private volatile double targetHood = 0.0;
    private volatile double targetRecoil = 0.0;
    private volatile double distance = 0.0;
    private ElapsedTime regTimer = new ElapsedTime();
    private final int updateTime = 20;

    @Override //ts is good practice to comment that you are overriding the thread class stuff. You need to in order to run stuff and you need to call (variable that is a thread).start() to start running the thread
    public void run(){
        while (running){
            if (regTimer.milliseconds() > updateTime){
                targetVel = getGoodShootVel(distance);
                targetHood = hoodPosSet(distance);
                targetRecoil = getRecoil(distance);
            }
            try {
                Thread.sleep(10); //something called cpu spinning
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    public double vel(){
        return targetVel;
    }
    public double hoo(){
        return targetHood;
    }
    public double rec(){
        return targetRecoil;
    }
    public void setDis(double dist){
        distance = dist;
    }
    public double getGoodShootVel(double distanceFromGoal){
        return 0.0000145 * Math.pow(distanceFromGoal, 4) - 0.00584813 * Math.pow(distanceFromGoal, 3) + 0.834897 * Math.pow(distanceFromGoal, 2) - 45.38315 * Math.pow(distanceFromGoal, 1) + 2000.07059;
    }
    public double hoodPosSet(double distanceFromGoal){
        return  -Math.pow(10, -9) * 2.0571 * Math.pow(distanceFromGoal, 4) - Math.pow(10, -7)*8.57305 * Math.pow(distanceFromGoal, 3) + 0.000313995 * Math.pow(distanceFromGoal, 2) - 0.0237158 * Math.pow(distanceFromGoal, 1) + 0.93;
    }
    public double getRecoil(double distanceFromGoal){
        return  -Math.pow(10, -9) * 5.66719 * Math.pow(distanceFromGoal, 4) + 0.00000199279 * Math.pow(distanceFromGoal, 3) -0.00024284 * Math.pow(distanceFromGoal, 2) +0.0127555 * Math.pow(distanceFromGoal, 1) -0.233045;
    }

    public void stopThread(){
        running = false;
    }
}