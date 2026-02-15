// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.time.chrono.ThaiBuddhistChronology;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.MsvcRuntimeException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Lib.Utils.BallLogic;
import frc.robot.Subsystems.ObjectDetection;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindThroughBalls extends Command {
  ArrayList<Pose2d> points;
  ArrayList<Pose2d> properPoints;
  ArrayList<RotationTarget> rotations;
  int numWayPoints;
  SwerveSubsystem mSwerve;
  PathPlannerPath path;
  Command followPath;
  int maxIndex = 0;

  /** Creates a new PathfindThroughBalls. */
  public PathfindThroughBalls(ArrayList<Pose2d> points, int numWayPoints, SwerveSubsystem mSwerve, ObjectDetection mDetection) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.points = points;
    this.numWayPoints = numWayPoints;
    this.mSwerve = mSwerve;

    addRequirements(mSwerve, mDetection);

    properPoints = new ArrayList<>();
    rotations = new ArrayList<>();
    maxIndex = points.size() < numWayPoints ? points.size() : numWayPoints;//maxIndex is which ever number is smaller
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //checks to make sure that there are points to pathfind through, if not it doesn't make a path
    if(points.size() > 0){
      //rearanges the random assortment of ball points to an organized and effiecent order based on which ball is next closest
      properPoints = BallLogic.rearangePoints(new Pose2d(SwerveSubsystem.poseEstimator.getEstimatedPosition().getTranslation(), SwerveSubsystem.getgyro0to360(0)), points, maxIndex + 1);

      //gets the max distance of the entire path
      double maxDistance = 0;
      for(int i = 1; i < properPoints.size(); i++){
        maxDistance += PhotonUtils.getDistanceToPose(properPoints.get(i - 1), properPoints.get(i));
      }

      //adds rotation targets to the path so the intake will always face the point it is driving to
      for(int i = 1; i < properPoints.size(); i++){
        //gets the distance of the point from the start
        double distanceFromStart = PhotonUtils.getDistanceToPose(properPoints.get(0), properPoints.get(i));
        //adds the rotation target at the percentage of the path that the ball is at
        rotations.add(new RotationTarget(i/properPoints.size(), properPoints.get(i).getRotation()));
        //rotations.add(new RotationTarget((distanceFromStart/maxDistance), properPoints.get(i).getRotation()));
        SmartDashboard.putNumber("percentage of the path that the rotation target is at", distanceFromStart/maxDistance);
      }
      SmartDashboard.putNumber("Size of properpoints", properPoints.size());

      //creates a new list of waypoints for the path
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(properPoints);

      SmartDashboard.putNumber("size of waypoints", waypoints.size());

      //contructs a path out of the waypoints, rotations, tele path constraints, and a goal end state
      //path = new PathPlannerPath(waypoints, SwerveConstants.telePathConstraints, null, new GoalEndState(0, properPoints.get(maxIndex + 1).getRotation()));
      path = new PathPlannerPath(waypoints, rotations, new ArrayList<>(), new ArrayList<>(), new ArrayList<>(), SwerveConstants.telePathConstraints, null, new GoalEndState(0, properPoints.get(maxIndex + 1).getRotation()), false);

      path.preventFlipping = true;//prevent flipping the path based on alliance color

      SwerveSubsystem.field.getObject("path").setPoses(path.getPathPoses());//log the path to the field2d
      
      followPath = AutoBuilder.followPath(path);//create the command to follow the path
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //checks to make sure that there is a path to run, if not it runs no path
    if(path != null){
      CommandScheduler.getInstance().schedule(followPath);//schedules the command to follow the path
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //cancels the path if interrupted (but it doesn't work :( )
    if(path != null){
      followPath.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
