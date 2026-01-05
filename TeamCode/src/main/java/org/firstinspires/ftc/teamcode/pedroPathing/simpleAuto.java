
//These are just a bunch of imports that you dont have to worry about
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous(name = "simpleAuto", group = "Autonomous")
public class simpleAuto extends OpMode {


    private Follower follower; //this guy just kinda executes the paths type stuff yk
    //This is core of pedropathing. This basically executes the paths (takes in which position you want to go to and travels there


    private Timer pathTimer, opmodeTimer; //Path timer can be used in the autonomousPathUpdate just to see if one of the paths failed or something
    //You dont have to worry about this really. This is kind of just like a failsafe to timeout the robot if something goes wrong

    private int pathState; //just an int used later in autonomousPathUpdate for each of the cases (tells which path to do)
    //Basically this is telling the robot which path it should be on (which position on the field it should go towards


    //These Are the Positions on the Field.
    //The x direction is forward and backward for pedropathing and the y is the sideways movement
    //the math.toRadians() thing is the angle that the robot should be facing on the field 0 degrees is the facing forward.
    // 90 degrees is facing to the right and -90 degrees is facing to the left

    private final Pose startPosition = new Pose(4, -60, Math.toRadians(0));
    private final Pose scorePosition = new Pose(82, -60, Math.toRadians(47));



    //These are the different paths that we want to execute.
    // This is basically just something that tells the follower where it should start and where it should end\
    private PathChain firstPosition, secondPosition;

    //This is the motor that controls the shooting mechanism. we just call it shooter because it controls the shooter but it can be named anything same with all of the other variables in pink
    DcMotor shooter;


    public void buildPaths() {
        firstPosition=follower.pathBuilder()
                .addPath(new BezierLine(startPosition, scorePosition))
                .setLinearHeadingInterpolation(startPosition.getHeading(), scorePosition.getHeading())
                .setTimeoutConstraint(1000)
                .build();
        secondPosition=follower.pathBuilder()
                .addPath(new BezierLine(scorePosition, startPosition))
                .setLinearHeadingInterpolation(scorePosition.getHeading(), startPosition.getHeading())
                .setTimeoutConstraint(3000)
                .build();

    }
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(firstPosition);
                setPathState(1);
                shooter.setPower(-1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(secondPosition,true);
                    setPathState(1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() { //this runs constantly during auto and we just update the position of the follower and check if it is still busy and cycle through each case
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        shooter = hardwareMap.get(DcMotor.class, "shoot1");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPosition);

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}