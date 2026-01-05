package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "DumbBlueAuto", group = "Autonomous")
public class DumbBlueAuto extends OpMode {
    DcMotor shooter, shooter2, intake, intake1;
    CRServo spindexRoter, up, hood;
    Servo push, turretRight, turretLeft;
    DcMotor FR, BR, FL, BL;
    private float timeSave;
    int i = 0;
    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotor.class, "shoot1");
        shooter2 = hardwareMap.get(DcMotor.class, "shoot2");
        push = hardwareMap.get(Servo.class, "push");
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        spindexRoter = hardwareMap.get(CRServo.class, "spindexRoter");
        //up = hardwareMap.get(CRServo.class, "intakeGate");
        //hood = hardwareMap.get(CRServo.class, "shooterHood");
        FR = hardwareMap.dcMotor.get("rf");
        BR = hardwareMap.dcMotor.get("rr");
        FL = hardwareMap.dcMotor.get("lf");
        BL = hardwareMap.dcMotor.get("lr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");

    }

    @Override
    public void loop() {
        if(i<1){
            push.setPosition(.85);
            shooter.setPower(-1);
            shooter2.setPower(1);
            intake.setPower(1);
            intake1.setPower(-1);
            FR.setPower(-0.75);
            BR.setPower(-0.75);
            FL.setPower(0.75);
            BL.setPower(0.75);
            timeSave=System.currentTimeMillis();
            //while(System.currentTimeMillis() < timeSave + 500){;}
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            FR.setPower(-0);
            BR.setPower(-0);
            FL.setPower(-0);
            BL.setPower(-0);
            timeSave=System.currentTimeMillis();
            //while(System.currentTimeMillis() < timeSave + 500){;}
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            push.setPosition(0.75);
            timeSave=System.currentTimeMillis();
            //while(System.currentTimeMillis() < timeSave + 500){;}
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            spindexRoter.setPower(-1);
            timeSave=System.currentTimeMillis();
            //while(System.currentTimeMillis() < timeSave + 2000){;}
            try {
                Thread.sleep(4500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            FR.setPower(-0.75);
            BR.setPower(-0.75);
            timeSave=System.currentTimeMillis();
            //while(System.currentTimeMillis() < timeSave + 250){;}
            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            FR.setPower(-0);
            BR.setPower(-0);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            FR.setPower(-.75);
            BR.setPower(-.75);
            FL.setPower(.75);
            BL.setPower(.75);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            FR.setPower(-.0);
            BR.setPower(-.0);
            FL.setPower(.0);
            BL.setPower(.0);
            shooter.setPower(0);
            shooter2.setPower(0);
            intake.setPower(0);
            intake1.setPower(0);
            spindexRoter.setPower(0);
            push.setPosition(.85);
            i+=1;
        }

    }
}
