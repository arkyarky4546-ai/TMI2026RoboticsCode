package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.servo720Rot;

public class intakeShoot {

    //DcMotors
    private DcMotorEx intakeMotor1, intakeMotor2, shootMotor1, shootMotor2;

    //my spindex rotator class. look at servo720Rot
    private servo720Rot spindexer;

    //doubles
    private double shootPower;
    private double TargetVelocity = 0;
    private double distance;

    public static double Kp=0.0047;
    public static double Ki=0.0004;
    public static double Kd=0;
    public static double Kf=0;

    //ints
    private int shootMode = 1;
    private int intakeMode = 0;
    private int arrayShootIntakeTrack;

    //various timers for delaying stuff (super useful in a lot of scenarios)
    private ElapsedTime PIDtimer = new ElapsedTime();

    public intakeShoot(HardwareMap hardwareMap, String intake1, String intake2, String shoot1, String shoot2, String servoName, String servoName2, String distance1, String distance2, String distance3, String distance4, String distance5, String distance6, String distance7){
        //constructor this is where everything is initialized
        intakeMotor1 =  hardwareMap.get(DcMotorEx.class, intake1);
        intakeMotor2 =  hardwareMap.get(DcMotorEx.class, intake2);

        shootMotor1 =  hardwareMap.get(DcMotorEx.class, shoot1);
        shootMotor2 =  hardwareMap.get(DcMotorEx.class, shoot2);

        //my custom class takes all of these variables
        spindexer = new servo720Rot(hardwareMap, servoName, servoName2, distance1, distance2, distance3, distance4, distance5, distance6, distance7);

    }
    public void update(double intakePower, double integralsum, double Lasterror, int pathstate, Telemetry telemetry){
        if(pathstate == 16) {
            shootPower = 0;
        }
        else {
            shootPower = PIDControl(getGoodVel(distance),shootMotor2.getVelocity(), integralsum, Lasterror);
            TargetVelocity = getGoodVel(distance);
        }
        shootsetPower(shootPower);
        intakesetPower(intakePower);
        if(spindexer.getDisMain() < 3){
            spindexer.sSP(spindexer.getFree(0, spindexer.getPos()),0);
        }
    }
    public double PIDControl(double reference, double state, double Integralsum, double lasterror){
        double error=reference-state;
        double dt = PIDtimer.seconds();
        Integralsum+=error*dt;
        double derivative=(error-lasterror)/dt;
        lasterror=error;
        PIDtimer.reset();
        return (error*Kp)+(derivative*Kd)+(Integralsum*Ki)+(reference*Kf);
    }
    public double getGoodVel(double dis){
        return 0.0000189394*(dis*dis*dis*dis) - 0.00598485*(dis*dis*dis) + 0.70947*(dis*dis) - 34.90476*(dis) + 1655.01299;
    }
    public void shootsetPower(double power){
        shootMotor1.setPower(-power);
        shootMotor1.setPower(power);
    }
    public void intakesetPower(double power){
        intakeMotor1.setPower(power);
        intakeMotor2.setPower(power);
    }
}
