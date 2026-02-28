package org.firstinspires.ftc.teamcode;

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

    private static final double SERVO_RANGE_DEGREES = 320; //361.0;
    private static final double CENTER_POSITION = 0.11;

    private double targetX = 0.0;
    private double targetY = 0.0;

    private double currentServoPosition = CENTER_POSITION;

    public AutoAimTurret(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
        servoLeft = hardwareMap.get(Servo.class, leftServoName);
        servoRight = hardwareMap.get(Servo.class, rightServoName);
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

        targetPos = Range.clip(targetPos, 0.0, 1.0);

        servoLeft.setPosition(1-targetPos);
        servoRight.setPosition(1-targetPos);

        currentServoPosition = targetPos;
    }

    public void setManualPosition(double position) {
        currentServoPosition = Range.clip(position, 0.0, 1.0);
        servoLeft.setPosition(currentServoPosition);
        servoRight.setPosition(currentServoPosition);
    }
}