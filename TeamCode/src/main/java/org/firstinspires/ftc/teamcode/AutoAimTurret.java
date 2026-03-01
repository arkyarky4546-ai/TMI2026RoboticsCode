package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class AutoAimTurret {

    private Servo servoLeft;
    private Servo servoRight;

    private static final double SERVO_RANGE_DEGREES = 320; //320?;
    private static final double CENTER_POSITION = 0.73;

    private double targetX = 0.0;
    private double targetY = 0.0;

    private double currentServoPosition = CENTER_POSITION;

    private Timer lastManual = new Timer();
    boolean autoAiming;


    public AutoAimTurret(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
        servoLeft = hardwareMap.get(Servo.class, leftServoName);
        servoRight = hardwareMap.get(Servo.class, rightServoName);
        autoAiming = true;
        servoLeft.setPosition(CENTER_POSITION);
        servoRight.setPosition(CENTER_POSITION);
    }

    public void setTargetCoordinates(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void update(Follower follower, Telemetry telemetry) {
            if (follower == null) return;
            Pose robotPose = follower.getPose();
            double robotX = robotPose.getX();
            double robotY = robotPose.getY();
            double robotHeadingRad = robotPose.getHeading();
            telemetry.addData("x = ", robotX);
            telemetry.addData("y = ", robotY);
            telemetry.addData("head = ", Math.toDegrees(robotHeadingRad));
            double angleToTargetRad = Math.atan2(targetY - robotY, targetX - robotX);

            double relativeAngleRad = angleToTargetRad - robotHeadingRad;

            relativeAngleRad = AngleUnit.normalizeRadians(relativeAngleRad);

            double relativeAngleDeg = Math.toDegrees(relativeAngleRad);

            double targetPos = (relativeAngleDeg / SERVO_RANGE_DEGREES) + CENTER_POSITION;

            if(targetPos > 1){
                targetPos -= 360/SERVO_RANGE_DEGREES;
            }

            targetPos = Range.clip(targetPos, 0.0, 1.0);

            servoLeft.setPosition(targetPos); //took off 1-targetpos
            servoRight.setPosition(targetPos);

            currentServoPosition = targetPos;
    }

    public void setManualPosition(double position) {
        currentServoPosition = Range.clip(position, 0.0, 1.0);
        servoLeft.setPosition(currentServoPosition);
        servoRight.setPosition(currentServoPosition);
    }

    public void setModeRed(){
        targetX = 138.0;
        targetY = -138.0;
        //CENTER_POSITION = .63;
    }

    public void setModeBlue(){
        targetX = 138.0;
        targetY = 0.0;
        //CENTER_POSITION = .89;
    }

    public void manualLeft(){
        if((servoLeft.getPosition() > 0.15) && (lastManual.getElapsedTime() > 100)){
            double toBeSetPos = servoLeft.getPosition() - 0.01;
            servoLeft.setPosition(toBeSetPos);
            servoRight.setPosition(toBeSetPos);
            lastManual.resetTimer();
        }
    }

    public void manualRight(){
        if((servoLeft.getPosition() < 0.85) && (lastManual.getElapsedTime() > 100)){
            double toBeSetPos = servoLeft.getPosition() + 0.01;
            servoLeft.setPosition(toBeSetPos);
            servoRight.setPosition(toBeSetPos);
            lastManual.resetTimer();
        }
    }

    public void setCenter(){
        servoLeft.setPosition(CENTER_POSITION);
        servoRight.setPosition(CENTER_POSITION);

    }

}