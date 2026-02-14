// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class ObjectDetection extends SubsystemBase {
  private static double[] ballAngles;
  private static double[] ballDistances;
  private static int numFuel;
  private static boolean plotBalls = false;
  public static NetworkTable fuelCV;
  public static ArrayList<Pose2d> ballPoses = new ArrayList<>();
  /** Creates a new ObjectDetection. */
  public ObjectDetection() {
    fuelCV = NetworkTableInstance.getDefault().getTable("fuelCV");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(fuelCV != null){
      handleFuelDetection();
    }
    
    //SmartDashboard.putNumber("ball angle", ballAngles[0]);
    //SmartDashboard.putNumber("ball distance", ballDistances[0]);
  }

  //TODO: make it so when we calculate the position of the ball we also set the rotational value of the Pose2d to be the angle from the robot to the ball(don't include the gyro)
  private static void handleFuelDetection(){
    numFuel = (int) fuelCV.getEntry("number_of_fuel").getInteger(0);
    
    if(numFuel > 0){
      plotBalls = true;
      ballAngles = fuelCV.getEntry("yaw_radians").getDoubleArray(ballAngles);
      ballDistances = fuelCV.getEntry("distance").getDoubleArray(ballDistances);
    }else{
      plotBalls = false;
    }

    if(plotBalls){
      for(int i = 0; i < ballAngles.length && i < ballDistances.length; i++){
        ballPoses.add(getBallPose(ballAngles[i], ballDistances[i]));//adds the pose to an array list of poses that can be later iterated through 

        if(ballPoses.size() > numFuel){
          for(int k = 0; k < ballPoses.size() - numFuel; k++){//removes any ball poses that remained from balls out of frame
            ballPoses.remove(numFuel);
          }
        }

        for(Pose2d ballPose : ballPoses){
          FieldObject2d ball = SwerveSubsystem.field.getObject("ball" + i);
          ball.setPose(ballPose);//gets the pose of the ball using the angle and distance at the current index
        }
      }
    }else{
      ballPoses.removeAll(ballPoses);//gets rid of all poses when there are no balls
    }
  }

  public static Pose2d getBallPose(double ballAngle, double ballDistance){
    double xdis = Math.cos(ballAngle) * ballDistance;//gets the ball's x distance from the camera
    double ydis = Math.sin(ballAngle) * ballDistance;//gets the ball's y distance from the camera

    double xWorldDis = Math.cos(SwerveSubsystem.getgyro0to360(180).getRadians() + ballAngle) * ballDistance;
    double yWorldDis = Math.sin(SwerveSubsystem.getgyro0to360(180).getRadians() + ballAngle) * ballDistance;

    Pose2d ballPoseRelativeToCamera = new Pose2d(xdis, ydis, new Rotation2d()); //ball pose relative to the camera

    Pose2d ballPoseRelativeToRobot = new Pose2d(ballPoseRelativeToCamera.getX() + VisionConstants.objectDetectionRobotToCamera.getX(), //ball pose relative to the center of the robot
          ballPoseRelativeToCamera.getY() + VisionConstants.objectDetectionRobotToCamera.getY(), new Rotation2d());

    Pose2d ballPoseRelativeToField = new Pose2d(SwerveSubsystem.detectionCamToField().getX() + xWorldDis, //ball pose relative to the field
          SwerveSubsystem.detectionCamToField().getY() + yWorldDis, 
          Rotation2d.fromDegrees(SwerveSubsystem.getgyro0to360(0).getDegrees() - ballAngle));/* <-- gives the ball a rotation from the intakes pov, 
            this is used later in the PathFindThroughBalls to find the optimal ending rotation 
            and so throughout the path the intake will be facing the balls and not some arbitrary number*/

    return ballPoseRelativeToField;
  }

}
