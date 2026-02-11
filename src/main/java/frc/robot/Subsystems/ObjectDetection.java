// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.google.flatbuffers.FlatBufferBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class ObjectDetection extends SubsystemBase {
  private static double ballAngle;
  private static double ballDistance;
  private static int numFuel;
  private static boolean plotBalls = false;
  public static NetworkTable fuelCV;
  /** Creates a new ObjectDetection. */
  public ObjectDetection() {
    fuelCV = NetworkTableInstance.getDefault().getTable("fuelFRC");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    handleFuelDetection();

    SmartDashboard.putNumber("ball angle", ballAngle);
    SmartDashboard.putNumber("ball distance", ballDistance);
  }

  private static void handleFuelDetection(){
    numFuel = (int) fuelCV.getEntry("number_of_fuel").getInteger(0);
    
    if(numFuel > 0){
      plotBalls = true;
      ballAngle = fuelCV.getEntry("yaw_radians").getDouble(0);
      ballDistance = fuelCV.getEntry("distance").getDouble(0);
    }else{
      plotBalls = false;
      ballAngle = 0;
      ballDistance = 0;
    }

    if(plotBalls){
      for(int i = 0; i < numFuel; i++){
        FieldObject2d ball = SwerveSubsystem.field.getObject("ball");
        ball.setPose(getBallPose(ballAngle, ballDistance));//change this to whatever the index is in the array of angles and distances
      }
    }
  }

  //TODO: make a method that, like the turret position calculator, calculates the position and rotation of the camera relative to the field and use that in this method to calculate the position of the ball.
  public static Pose2d getBallPose(double ballAngle, double ballDistance){
    double xdis = Math.cos(ballAngle) * ballDistance;//gets the ball's x distance from the camera
    double ydis = Math.sin(ballAngle) * ballDistance;//gets the ball's y distance from the camera

    Pose2d ballPoseRelativeToCamera = new Pose2d(xdis, ydis, new Rotation2d()); //ball pose relative to the camera

    Pose2d ballPoseRelativeToRobot = new Pose2d(ballPoseRelativeToCamera.getX() + VisionConstants.objectDetectionRobotToCamera.getX(), //ball pose relative to the center of the robot
          ballPoseRelativeToCamera.getY() + VisionConstants.objectDetectionRobotToCamera.getY(), new Rotation2d());

    Pose2d ballPoseRelativeToField = new Pose2d(SwerveSubsystem.poseEstimator.getEstimatedPosition().getX() + ballPoseRelativeToRobot.getX(), //ball pose relative to the field
          SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() + ballPoseRelativeToRobot.getY(), new Rotation2d());

    return ballPoseRelativeToField;
    
  }

}
