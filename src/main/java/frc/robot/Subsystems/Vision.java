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
    //Iterates through each limelight and sets the pipeline index to april tag index
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
    //iterates through the number of limelights in the limelightNames array
    for(String i : limelightNames){

      //all of the tags distance from the camera
      double totalDistanceToTags = 0;

      //tells Limelight Helpers what the robot's rotation is
      LimelightHelpers.SetRobotOrientation(i, SwerveSubsystem.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      //Calculates the robot's field relative position for the camera using Mega Tag 2
      LimelightHelpers.PoseEstimate limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(i);
      
      //Makes sure that the pose returned from Mega Tag 2 isn't null before attempting to do everything
      if(limelightPose != null){
        double limelightAmountOfTags = limelightPose.tagCount;//gets the number of tags in the frame
        // makes sure that it isn't 0 (this is pretty much redundant because if there are no tags then the limelightpose is going to be null)
        if(limelightAmountOfTags != 0){
          double latency = LimelightHelpers.getLatency_Capture(i) + LimelightHelpers.getLatency_Pipeline(i);//gets the latency
          double limelightTimestamp = Timer.getFPGATimestamp() - (latency / 1000.0);//calculates the timestamp of the frame using the latency and the FPGA timestamp

          //iterates through the amount of tags in the frame
          for(int k=0; k<limelightAmountOfTags; k++){
            totalDistanceToTags += LimelightHelpers.getRawFiducials(i)[k].distToCamera;//adds their distances to the totalDistanceToTags variable
          }

          //calculates the average distance to all tags in the frame
          double avgDistToTags = totalDistanceToTags/limelightAmountOfTags;
          //Some fancy Mechanical Advantage stuff that makes the standard deviation really nice (idk why it works it just does and it amazing)
          double xyStdVal = VisionConstants.stdCoefficient * Math.pow(avgDistToTags, 1.2) / Math.pow(limelightAmountOfTags, 2);

          
          SwerveSubsystem.poseEstimator.addVisionMeasurement(limelightPose.pose, limelightTimestamp, VecBuilder.fill(xyStdVal, xyStdVal, Double.POSITIVE_INFINITY));
          /*
          Adds the pose as a vision measurement to the pose estimator in the Swerve Subsystem. 
          Uses the calculated standard deviation measurments for translation(x & y) 
          and uses positive infinity as the rotational standard deviation so that the pose estimator won't attempt to use 
          the rotational value calculated by the camera and will instead use the gyro's value.
          */
          }
      }
    }
  }
}
