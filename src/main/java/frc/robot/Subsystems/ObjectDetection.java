// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Lib.Utils.Ball;

public class ObjectDetection extends SubsystemBase {
  public double[] ballAngles, ballDistances, weights, ballx, bally, sortedWeights;
  private int numFuel;
  private boolean plotBalls = false;
  public NetworkTable fuelCV;
  public ArrayList<Pose2d> ballPoses = new ArrayList<>();
  public ArrayList<Ball> balls = new ArrayList<>();
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

    SmartDashboard.putNumber("Ball Poses Size", ballPoses.size());
    if(ballx.length > 0){
      sortBalls(ballx, bally, weights);
    }
    
    //SmartDashboard.putNumber("ball angle", ballAngles[0]);
    //SmartDashboard.putNumber("ball distance", ballDistances[0]);
  }

  private void sortBalls(double[] x, double[] y, double[] weights){
    for(int i = 0; i<x.length; i++){
      balls.add(i, new Ball(x[i], y[i], weights[i]));
    }
    Comparator<Ball> ballComparator = Comparator.comparing(Ball::getWeight);
    Collections.sort(balls, ballComparator);
  }

  //handles all detection of fuel and adds them to a list of current fuel in the frame
  private void handleFuelDetection(){
    numFuel = (int) fuelCV.getEntry("number_of_fuel").getInteger(0);//gets the total number of fuel
    
    //if there are fuel in the frame then we set plotBalls to true and start getting the important values from said balls
    if(numFuel > 0){
      plotBalls = true;
      ballAngles = fuelCV.getEntry("yaw_radians").getDoubleArray(ballAngles);//gets an array of angles from the camera to each ball
      ballDistances = fuelCV.getEntry("distance").getDoubleArray(ballDistances);//gets and array of distances from the camera to each ball
      weights = fuelCV.getEntry("weights").getDoubleArray(weights);
      ballx = fuelCV.getEntry("ball_position_x").getDoubleArray(ballx);
      bally = fuelCV.getEntry("ball_position_y").getDoubleArray(bally);
      
    }else{
      plotBalls = false;
    }

    //if plotBalls is true then we plot the balls (who would have guessed :) )
    if(plotBalls && ballx != null && bally != null){
      //iterates through each index that appears both in the distances and in the angles arrays
      for(int i = 0; i < ballAngles.length && i < ballDistances.length; i++){
        //ballPoses.add(new Pose2d(new Translation2d(ballx[i], bally[i]), new Rotation2d()));//adds the pose to an array list of poses that can be later iterated through 

        //if there are more balls in the stored poses than in the frame we remove them

        //Logs all of the balls to the field2d
        for(int k = 0; k<ballPoses.size(); k++){
          FieldObject2d ball = SwerveSubsystem.field.getObject("ball" + k);
          ball.setPose(new Pose2d(ballx[k], bally[k], new Rotation2d()));//gets the pose of the ball using the angle and distance at the current index
        }
      }
    }else{
      ballPoses.removeAll(ballPoses);//gets rid of all poses when there are no balls
    }
  }

  //Uses an angle and a distance to calculate the position of a detected ball relative to the field
  public Pose2d getBallPose(double ballAngle, double ballDistance){

    //gets the world x and y positions of the ball from the camera using the rotation of the robot (since the camera is static we can get the gyro value and add an offset to get the camera's angle)
    double xWorldDis = Math.cos(SwerveSubsystem.getgyro0to360(180).getRadians() + ballAngle) * ballDistance;
    double yWorldDis = Math.sin(SwerveSubsystem.getgyro0to360(180).getRadians() + ballAngle) * ballDistance;

    Pose2d ballPoseRelativeToField = new Pose2d(SwerveSubsystem.detectionCamToField().getX() + xWorldDis, //ball pose relative to the field
          SwerveSubsystem.detectionCamToField().getY() + yWorldDis, 
          Rotation2d.fromDegrees(SwerveSubsystem.getgyro0to360(0).getDegrees() - ballAngle));/* <-- gives the ball a rotation from the intakes pov, 
            this is used later in the PathFindThroughBalls to find the optimal ending rotation 
            and so throughout the path the intake will be facing the balls and not some arbitrary number*/

    return ballPoseRelativeToField;//returns the ball's pose on the field
  }

}
