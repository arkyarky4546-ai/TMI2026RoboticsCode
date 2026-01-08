package org.firstinspires.ftc.teamcode.revisedTeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

public class Drivetrain {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.5;
    private int mode;
    private final int RED = 1;
    private final int BLUE = 2;

    public Drivetrain(HardwareMap hardwareMap) {
        mode = 0;
        startingPose = new Pose(65, -97, Math.toRadians(0));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(8, -96))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();

        follower.startTeleopDrive();
    }

    public void update(double left_stick_y, double left_stick_x, double right_stick_x, boolean xWasPressed, boolean bWasPressed){
        follower.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -left_stick_y,
                    -left_stick_x,
                    -right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -left_stick_y * slowModeMultiplier,
                    -left_stick_x * slowModeMultiplier,
                    -right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (xWasPressed) {
            Pose targetPose = altitudeExist(follower.getPose());
            /*telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.addData("going to x: ", targetPose.getX());
            telemetry.addData("going to y: ", targetPose.getY());*/
            PathChain twoPointsOrSmthn = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), targetPose))
                    .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                    .build();
            follower.followPath(twoPointsOrSmthn);
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (bWasPressed || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

    }

    public Follower getFollower(){
        return follower;
    }

    public void switchSlowMode(){
        slowMode = !slowMode;
    }

    public Pose altitudeExist(Pose currentPose){
        double value = Math.abs(Math.abs(currentPose.getX()) + Math.abs(currentPose.getY()))/2;
        if(mode == RED) {
            return new Pose(value, -value, Math.toRadians(-47));
        }
        else if(mode == BLUE){
            return new Pose(value, -(144-value), Math.toRadians(47));
        }
        return new Pose(0, 0, Math.toRadians(0));
    }

    public void setModeRed(){
        mode = RED;
        startingPose = new Pose(65, -97, Math.toRadians(0));
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
    }

    public void setModeBlue(){
        mode = BLUE;
        startingPose = new Pose(65, -48, Math.toRadians(47));
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
    }

    //TODO: find actual positions (odometry pods are slightly off center)
    public void resetCurrentPose(){
        if(mode == RED){
            follower.setPose(new Pose(0, 0, Math.toRadians(0))); //far left corner of the field
        }
        else if(mode == BLUE){
            follower.setPose(new Pose(0, -144, Math.toRadians(0))); //far right corner of the field
        }
    }

    public double getDistanceFromGoal(){
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        if(mode == RED){
            return Math.sqrt((x + 144) * (x + 144) + (y - 144) * (y - 144)); //-144, 144
        }
        else if(mode == BLUE){
            return Math.sqrt((x + 0) * (x + 0) + (y - 144) * (y - 144)); //0, 144
        }
        else{
            return 0;
        }
    }

}