package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

//it's the same as the AutoAimTurret class but I added a safety in case the drivers want to aim manually
public class Turret {

    private Servo servoLeft;
    private Servo servoRight;

    private static final double SERVO_RANGE_DEGREES = 361.0;
    private static final double CENTER_POSITION = 0.5;

    private double targetX = 0.0;
    private double targetY = 0.0;

    private double currentServoPosition = CENTER_POSITION;
    private final double gearRatio =  1.0055911*1.11111111;//input / output 1.0055911

    private boolean autoAiming;
    private Timer lastManual = new Timer();

    public Turret(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
        servoLeft = hardwareMap.get(Servo.class, leftServoName);
        servoRight = hardwareMap.get(Servo.class, rightServoName);
        autoAiming = true;
    }

    public void setTargetCoordinates(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void update(Follower follower, Telemetry telemetry){
        if(autoAiming){
            this.updateAuto(follower, telemetry);
        }
    }

    public void updateAuto(Follower follower, Telemetry telemetry) {
        if (follower == null) return;
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeadingRad = robotPose.getHeading();
        telemetry.addData("Turret X", robotX);
        telemetry.addData("Turret Y", robotY);
        telemetry.addData("Turret Head", Math.toDegrees(robotHeadingRad));
        double angleToTargetRad = Math.atan2(targetY - robotY, targetX - robotX);

        double relativeAngleRad = angleToTargetRad - robotHeadingRad;
        relativeAngleRad = AngleUnit.normalizeRadians(relativeAngleRad);
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);
        double servoAngleNeeded = relativeAngleDeg * gearRatio/360;
        double targetPos = (servoAngleNeeded) + (1 - CENTER_POSITION);

        if(targetPos < 0){
            targetPos += 360/355*1.111111111;

        }
        targetPos = Range.clip(targetPos, 0.0, 1.0);
        servoLeft.setPosition(1 - targetPos);
        servoRight.setPosition(1 - targetPos);
        currentServoPosition = targetPos;
    }

    public void setManualPosition(double position) {
        currentServoPosition = Range.clip(position, 0.0, 1.0);
        servoLeft.setPosition(currentServoPosition);
        servoRight.setPosition(currentServoPosition);
    }

    public void switchMode(){
        autoAiming = !autoAiming;
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

    //TODO: confirm center values
    public void setTurretCenter(){
        autoAiming = false;
        servoLeft.setPosition(0.0);
        servoRight.setPosition(0.0);
    }

    public boolean isAutoAiming(){
        return autoAiming;
    }

    public void setModeRed(){
        targetX = 144.0;
        targetY = -144.0;
    }

    public void setModeBlue(){
        targetX = 144.0;
        targetY = 0.0;
    }
}