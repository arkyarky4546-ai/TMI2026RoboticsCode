package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

@TeleOp
public class PedroAutoAimTurret extends OpMode {
    private Servo servoLeft;
    private Servo servoRight;
    private static final double CENTER_POSITION = 0.5;
    private double servoRangeDegrees = 180;
    private double gearRatio = 1.0; //input / output
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double currentServoPosition = CENTER_POSITION;

    private Follower follower;

    @Override
    public void init() {
        this.servoRangeDegrees = servoRangeDegrees;
        this.gearRatio = gearRatio;
        servoLeft = hardwareMap.get(Servo.class, "turretLeft");
        servoRight = hardwareMap.get(Servo.class, "turretRight");

        servoLeft.setPosition(CENTER_POSITION);
        servoRight.setPosition(CENTER_POSITION);

        // --- PedroPathing follower ---
        follower = Constants.createFollower(hardwareMap);

        // Optional: set starting pose
        follower.setPose(new Pose(0, 0, 0));

        // Initial target (example: high basket)
        setTargetCoordinates(48, 48);

        telemetry.addLine("Turret Initialized");
    }

    @Override
    public void loop() {
        // Update PedroPathing localization
        follower.update();

        // Auto aim turret
        updateTurret(follower,telemetry);


        Pose pose = follower.getPose();
        // Telemetry
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Heading",
                Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

    public PedroAutoAimTurret(HardwareMap hardwareMap, String leftServoName, String rightServoName, double servoRangeDegrees, double gearRatio) {
        servoLeft = hardwareMap.get(Servo.class, leftServoName);
        servoRight = hardwareMap.get(Servo.class, rightServoName);
        this.servoRangeDegrees = servoRangeDegrees;
        this.gearRatio = gearRatio;
    }

    public void setTargetCoordinates(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void updateTurret(Follower follower, Telemetry telemetry) {
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
        double servoAngleNeeded = relativeAngleDeg * gearRatio;
        double targetPos = (servoAngleNeeded / servoRangeDegrees) + CENTER_POSITION;
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
}