// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class ObjectDetection extends SubsystemBase {
  private static double ballAngle;
  private static double ballDistance;
  public static NetworkTable fuelCV;
  /** Creates a new ObjectDetection. */
  public ObjectDetection() {
    fuelCV = NetworkTableInstance.getDefault().getTable("fuelFRC");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ballAngle = fuelCV.getEntry("yaw_radians").getDouble(0);
    ballDistance = fuelCV.getEntry("distance").getDouble(0);

    SmartDashboard.putNumber("ball angle", ballAngle);
    SmartDashboard.putNumber("ball distance", ballDistance);
  }

  public static Pose2d getBallPose(double ballAngle, double ballDistance){
    double xdis = Math.cos(ballAngle) * ballDistance;
    double ydis = Math.sin(ballAngle) * ballDistance;
    Pose2d ballPoseRelativeToCamera = new Pose2d(xdis, ydis, new Rotation2d());
    Pose2d ballPoseRelativeToRobot = new Pose2d(ballPoseRelativeToCamera.getX() + VisionConstants.objectDetectionRobotToCamera.getX(), //ball pose relative to the center of the robot
          ballPoseRelativeToCamera.getY() + VisionConstants.objectDetectionRobotToCamera.getY(),
          VisionConstants.objectDetectionRobotToCamera.getRotation());
    Pose2d ballPoseRelativeToField = new Pose2d(SwerveSubsystem.poseEstimator.getEstimatedPosition().getX() + ballPoseRelativeToRobot.getX(), //ball pose relative to the field
          SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() + ballPoseRelativeToRobot.getY(), 
          SwerveSubsystem.getgyro0to360(VisionConstants.objectDetectionRobotToCamera.getRotation().getDegrees()));
    return ballPoseRelativeToField;
    
  }

}
