package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private DcMotor FR, BR, FL, BL;


    public void init(HardwareMap hardwareMap) {

        FR = hardwareMap.dcMotor.get("rf");
        BR = hardwareMap.dcMotor.get("rr");
        FL = hardwareMap.dcMotor.get("lf");
        BL = hardwareMap.dcMotor.get("lr");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void arcadeDrive(double drive, double strafe, double twist) {
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i]))
                max = Math.abs(speeds[i]);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] /= max;
            }
        }

        FL.setPower(speeds[0]);
        FR.setPower(speeds[1]);
        BL.setPower(speeds[2]);
        BR.setPower(speeds[3]);
    }

/*
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;
        arcadeDrive(drive, -strafe, -twist);
    } */
}