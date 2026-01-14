package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

//@TeleOp
public class PedroAutoAimTurret {

    private Servo servoLeft;
    private Servo servoRight;
    private static final double CENTER_POSITION = 0.96;
    private double servoRangeDegrees = 230;
    private double gearRatio =  1.0055911*1.11111111;//input / output 1.0055911
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double currentServoPosition = CENTER_POSITION;
    private double turretZeroDegOffset=0;
    private final Pose startPose = new Pose(128,-26, Math.toRadians(47));
    private boolean auto = true;
    private double manualpos = 1.0;

    private Follower follower;

    /*@Override
    public void init() {
        servoLeft = hardwareMap.get(Servo.class, "turretLeft");
        servoRight = hardwareMap.get(Servo.class, "turretRight");

        servoLeft.setPosition(CENTER_POSITION);
        servoRight.setPosition(CENTER_POSITION);

        // --- PedroPathing follower ---
        follower = Constants.createFollower(hardwareMap);

        // Optional: set starting pose

        follower.setStartingPose(startPose);

        // Initial target
        setTargetCoordinates(144, 0);//blue goal

        telemetry.addLine("Turret Initialized");
    }

    @Override
    public void loop() {
        // Update PedroPathing localization
        follower.update();

        // Auto aim turret
        if (auto) {


            updateTurret(follower, telemetry);
        }

        if(gamepad1.dpad_up){
            auto = false;
            setManualPosition(manualpos);
        }
        if(gamepad1.dpad_down){
            auto = true;
        }
        if(gamepad1.dpad_left){
            manualpos -= 0.01;
        }
        if(gamepad1.dpad_right){
            manualpos += 0.01;
        }
        if(gamepad2.a){
            targetX=0;
            targetY=0;
        }
        if(gamepad1.a){
            targetX=144;//blue goal
            targetY=0;
        }
        if(gamepad1.b){
            targetX=144;//red goal
            targetY=-144;
        }
        if(gamepad1.x){
            auto = false;
            setManualPosition(0.0);
        }
        if(gamepad1.y){
            auto = false;
            setManualPosition(1.0);
        }
        // Telemetry
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Heading",
                Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("target X", targetX);
        telemetry.addData("target Y", targetY);
        telemetry.addData("gear Ratio", gearRatio);
        telemetry.addData("manualPos", manualpos);
        telemetry.update();
    }*/

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
}