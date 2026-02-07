// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  public static String[] limelightNames = {"limelight-left", "limelight-right"};
  /** Creates a new Vision. */
  public Vision() {
    for(String i : limelightNames){
      LimelightHelpers.setPipelineIndex(i, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePoseWithLimelight();
  }

  private static void updatePoseWithLimelight(){
    for(String i : limelightNames){

      double totalDistanceToTags = 0;
      LimelightHelpers.SetRobotOrientation(i, SwerveSubsystem.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(i);
      if(limelightPose != null){
        double limelightAmountOfTags = limelightPose.tagCount;
        if(limelightAmountOfTags != 0){
          double latency = LimelightHelpers.getLatency_Capture(i) + LimelightHelpers.getLatency_Pipeline(i);
          double limelightTimestamp = Timer.getFPGATimestamp() - (latency / 1000.0);


          for(int k=0; k<limelightAmountOfTags; k++){
            totalDistanceToTags += LimelightHelpers.getRawFiducials(i)[k].distToCamera;
          }


          double avgDistToTags = totalDistanceToTags/limelightAmountOfTags;
          double xyStdVal = VisionConstants.stdCoefficient * Math.pow(avgDistToTags, 1.2) / Math.pow(limelightAmountOfTags, 2);

          
          SwerveSubsystem.poseEstimator.addVisionMeasurement(limelightPose.pose, limelightTimestamp, VecBuilder.fill(xyStdVal, xyStdVal, Double.POSITIVE_INFINITY));
        }
      }
    }
  }
}
