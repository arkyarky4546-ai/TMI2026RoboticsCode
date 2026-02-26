package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.ShooterConstants;

public class shooterThread extends Thread {
    private volatile boolean running = true;
    private volatile Follower follow;
    private volatile double robHead;
    private volatile double Speed;
    private volatile double turret;
    private volatile double hood;
    private volatile Pose goal;
    private volatile Shooter shooter;
    private volatile Shooter.ShotParameters params;

    public shooterThread(Shooter shooter, Follower follower, Pose goalPose, double robotHeading) {
        this.shooter = shooter;
        this.follow = follower;
        this.goal = goalPose;
        this.robHead = robotHeading;
    }
    @Override
    //ts is good practice to comment that you are overriding the thread class stuff. You need to in order to run stuff and you need to call (variable that is a thread).start() to start running the thread
    public void run() {
        while (running) {
            params = shooter.calculateShotVectorAndUpdateTurret(robHead, goal, follow);
            hood = ShooterConstants.hoodDegreesToServoTicks(params.hoodAngle);
            Speed =ShooterConstants.getFlyWheelTicksFromVelocity(params.flywheelSpeed);
            turret = params.turretAngle;
            try {
                Thread.sleep(10); //something called cpu spinning
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    public void update(Follower follower, Pose goalPose, double robotHeading){
        follow = follower;
        goal = goalPose;
        robHead = robotHeading;
    }
    public double getHoodPos(){
        return hood;
    }
    public double getTurretPos(){
        return turret;
    }
    public double getSpeed(){
        return Speed;
    }
    public void stopThread(){
        running = false;
    }
}