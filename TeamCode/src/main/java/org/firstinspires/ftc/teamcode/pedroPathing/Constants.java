package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.87993) //chassis in kg
           .forwardZeroPowerAcceleration(-44.3382333333) //-45.5667 -38.02 -43.17 -44.278 (old was -42.06)
            .lateralZeroPowerAcceleration(-83.901) //-86.7 -77.14 -85.96 -90.84 -78.865 (very inconsistent - old was -82)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.025, 0)) //looked good to me
            .headingPIDFCoefficients(new PIDFCoefficients(1.12, 0, 0, 0.01)) //bumped p up from 1.1 to 1.12
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.00001,0.6,0)) //seemed fine
            .centripetalScaling(0.000499999)
    ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .2, 1);

    public static MecanumConstants driveConstants = new MecanumConstants() //not fixed :(
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75.5066666667) //67.16 74.99 76.72 74.81 (got rid of outlier - old was 77.73)
            .yVelocity(61.0833333333) //60.55 61.32 61.38 (old was 60.367)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.113188976)
            .strafePodX(2.147027559)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("allMyHomiesHateOctoQuad")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }




