package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import com.pedropathing.math.Vector;

public class Shooter {
    public static class ShotParameters {
        public volatile double hoodAngle;
        public volatile double flywheelSpeed;
        public volatile double turretAngle;

        public ShotParameters(double hoodAngle, double flywheelSpeed, double turretAngle) {
            this.hoodAngle = hoodAngle;
            this.flywheelSpeed = flywheelSpeed;
            this.turretAngle = turretAngle;
        }
    }
    public ShotParameters calculateShotVectorAndUpdateTurret(double robotHeading, Pose goalPose, Follower follower){ //pass in either goalpose blue or red
//        Vector robotToGoalVector = new Vector(
//                goalPose.getX() - follower.getPose().getX(),
//                goalPose.getY() - follower.getPose().getY()
//        );
        Vector robotToGoalVector = new Vector(
                Math.sqrt(Math.pow(goalPose.getX() - follower.getPose().getX(), 2) + Math.pow(goalPose.getY() - follower.getPose().getY(), 2)),
                Math.atan2(follower.getPose().getY() - goalPose.getY(), goalPose.getX() - follower.getPose().getX())
        );
        double g = 32.174 * 12; //gravity in inches/second
        //need to make a vector from the robot to the goal from follower I think
        double x = robotToGoalVector.getMagnitude() - ShooterConstants.Pass_Through_Radius;
        double y = ShooterConstants.Score_Height;
        double a = ShooterConstants.Score_Angle;
        //calculate initial launch components
        //assuming that the robot is not moving
        double hoodAngle = MathFunctions.clamp(Math.atan(2*y/x-Math.tan(a)), ShooterConstants.Hood_Min_Angle, ShooterConstants.Hood_Max_Angle);
        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle),2) * (x * Math.tan(hoodAngle) - y)));
        //get robot velocity and convert it into parallel and perpendicular components
        //Vector robotVelocity = new Vector(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent()); //just use follower for this
        Vector robotVelocity = new Vector(
                Math.sqrt(Math.pow(goalPose.getX() - follower.getPose().getX(), 2) + Math.pow(goalPose.getY() - follower.getPose().getY(), 2)),
                Math.atan2(goalPose.getX() - follower.getPose().getX(), goalPose.getY() - follower.getPose().getY())
        );
        robotVelocity = follower.poseTracker.getVelocity();
        //difference in heading from the robots velocity and the robot to goal vector
        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude(); //seems to be the robots velocity and finding which is in which way

        //velocity compensation variable
        double v2 = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time; //distance

        //recalc launch components
        hoodAngle = MathFunctions.clamp(Math.atan(v2 / nvr), ShooterConstants.Hood_Min_Angle, ShooterConstants.Hood_Max_Angle);
        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) -y)));
        //update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double turretAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() + turretVelCompOffset);
        if (turretAngle > 180) {
            turretAngle -= 360;
        }
        return new ShotParameters(Math.toDegrees(hoodAngle), flywheelSpeed, turretAngle);
    }

}
