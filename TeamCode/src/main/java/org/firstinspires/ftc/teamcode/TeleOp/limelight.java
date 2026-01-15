package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

//@TeleOp
public class limelight extends OpMode {
    Limelight3A limelight3A;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);
        //IMU imu = hardwareMap.get(IMU.class, "imu");
        //RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
        //        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        //imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }

    public void start(){
        limelight3A.start();
    }

    @Override
    public void loop() {
        //IMU imu = hardwareMap.get(IMU.class, "imu");
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        LLResult result = limelight3A.getLatestResult();
        assert result != null;
        if (result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();
            telemetry.addData("Tarea", result.getTa());
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("botPose", botPose.toString());
            //  telemetry.addData("Yaw", orientation.getYaw());
            //  telemetry.addData("Pitch", orientation.getPitch());
            //  telemetry.addData("Roll", orientation.getRoll());
        }
    }
}
