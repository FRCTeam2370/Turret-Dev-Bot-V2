// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib.Utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.SwerveSubsystem;

/** Add your docs here. */
public class TurretLogic {
    SwerveSubsystem mSwerve;
    BrentSolver brentSolver = new BrentSolver(0.00001,0.00001,0.00001);

    public TurretLogic(SwerveSubsystem mSwerve){
        this.mSwerve = mSwerve;
    }

    public Translation3d getAimPose(Translation3d targetPose, double launchSpeed){
        double robotFieldXVel = mSwerve.getRobotRelativeSpeeds().vxMetersPerSecond * Math.cos(SwerveSubsystem.getgyro0to360(0).getRadians()) - mSwerve.getRobotRelativeSpeeds().vyMetersPerSecond * Math.sin(SwerveSubsystem.getgyro0to360(0).getRadians());
        double robotFieldYVel = mSwerve.getRobotRelativeSpeeds().vyMetersPerSecond * Math.sin(SwerveSubsystem.getgyro0to360(0).getRadians()) + mSwerve.getRobotRelativeSpeeds().vxMetersPerSecond * Math.cos(SwerveSubsystem.getgyro0to360(0).getRadians());
        double robotSpeed = Math.sqrt(Math.pow(robotFieldXVel, 2) + Math.pow(robotFieldYVel, 2));

        double targetPoseRelativeToRobotX = targetPose.getX() - SwerveSubsystem.poseEstimator.getEstimatedPosition().getX();
        double targetPoseRelativeToRobotY = targetPose.getY() - SwerveSubsystem.poseEstimator.getEstimatedPosition().getY();

        double angleRobotVelocityToTarget = Math.atan2(robotFieldYVel, robotFieldXVel) - Math.atan2(targetPoseRelativeToRobotY, targetPoseRelativeToRobotX);

        double lateralOffsetVelocityConstant = robotSpeed * Math.sin(angleRobotVelocityToTarget) / Math.sqrt(Math.pow(targetPoseRelativeToRobotX,2) + Math.pow(targetPoseRelativeToRobotY, 2));
        double lateralOffsetVelocityX = lateralOffsetVelocityConstant * targetPoseRelativeToRobotY;
        double lateralOffsetVelocityY = lateralOffsetVelocityConstant * -targetPoseRelativeToRobotX;

        double flattenedInitialVel = Math.sqrt(Math.pow(launchSpeed, 2) - Math.pow(robotSpeed * Math.sin(angleRobotVelocityToTarget), 2));
        double flattenedRobotVel = robotSpeed * Math.cos(angleRobotVelocityToTarget);

        double flattenedTargetPoseX = Math.sqrt(Math.pow(targetPoseRelativeToRobotX, 2) + Math.pow(targetPoseRelativeToRobotY, 2));
        double flattenedTargetPoseY = targetPose.getZ() - TurretConstants.TurretVerticalOffset;

        double zeroOfTheDerivativeOfTheDesiredAngle = Double.NaN;
        try{
            zeroOfTheDerivativeOfTheDesiredAngle = brentSolver.findRoot((double theta)-> Math.pow(flattenedInitialVel,2)*flattenedTargetPoseX*Math.cos(2*theta) - flattenedInitialVel*flattenedTargetPoseX*flattenedRobotVel*Math.cos(theta) + flattenedTargetPoseY*flattenedInitialVel*Math.sin(theta), TurretConstants.TurretMinAngle.getRadians(), TurretConstants.TurretMaxAngle.getRadians());
        }catch(Exception e){
            System.out.println(e);
        }
        
        double trueAngle = 0;
        if(Double.toString(zeroOfTheDerivativeOfTheDesiredAngle) != "NaN"){
            System.out.println("---------------" + Double.toString(zeroOfTheDerivativeOfTheDesiredAngle) + "-----------------");
            try{
                trueAngle = brentSolver.findRoot((double theta)-> flattenedInitialVel*flattenedTargetPoseX*(flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*Math.sin(theta) - 4.905*Math.pow(flattenedTargetPoseX,2) - (flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*flattenedTargetPoseY, zeroOfTheDerivativeOfTheDesiredAngle, TurretConstants.TurretMaxAngle.getRadians());
            }catch(Exception e){
                try{
                    trueAngle = brentSolver.findRoot((double theta)-> flattenedInitialVel*flattenedTargetPoseX*(flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*Math.sin(theta) - 4.905*Math.pow(flattenedTargetPoseX,2) - (flattenedInitialVel*Math.cos(theta) - flattenedRobotVel)*flattenedTargetPoseY, TurretConstants.TurretMinAngle.getRadians(), zeroOfTheDerivativeOfTheDesiredAngle); 
                }catch(Exception E){
                    System.out.println("Cry" + E);
                }
            }
        }
        
        double angleToTarget = Math.atan2(targetPoseRelativeToRobotY, targetPoseRelativeToRobotX);

        double vUnajustedX = Math.cos(trueAngle) * Math.cos(angleToTarget) * flattenedInitialVel;
        double vUnajustedY = Math.cos(trueAngle) * Math.sin(angleToTarget) * flattenedInitialVel;
        double vUnajustedZ = Math.sin(trueAngle) * flattenedInitialVel;

        return new Translation3d(vUnajustedX - lateralOffsetVelocityX + SwerveSubsystem.poseEstimator.getEstimatedPosition().getX(), 
            vUnajustedY - lateralOffsetVelocityY + SwerveSubsystem.poseEstimator.getEstimatedPosition().getY(), 
            vUnajustedZ + TurretConstants.TurretVerticalOffset);
    }
}
