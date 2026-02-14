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
    maxIndex = points.size() < numWayPoints ? points.size() : numWayPoints;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(points.size() > 0){
      properPoints = BallLogic.rearangePoints(/*SwerveSubsystem.poseEstimator.getEstimatedPosition(),*/new Pose2d(SwerveSubsystem.poseEstimator.getEstimatedPosition().getTranslation(), SwerveSubsystem.getgyro0to360(180)), points, maxIndex + 1);

      double maxDistance = 0;
      for(int i = 1; i < properPoints.size(); i++){
        maxDistance += PhotonUtils.getDistanceToPose(properPoints.get(i - 1), properPoints.get(i));
      }

      for(int i = 1; i<properPoints.size(); i++){
        double distanceFromStart = PhotonUtils.getDistanceToPose(properPoints.get(0), properPoints.get(i));
        rotations.add(new RotationTarget((distanceFromStart/maxDistance), properPoints.get(i).getRotation()));
      }
      SmartDashboard.putNumber("Size of properpoints", properPoints.size());

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(properPoints);

      //path = new PathPlannerPath(waypoints, SwerveConstants.telePathConstraints, null, new GoalEndState(0, properPoints.get(maxIndex + 1).getRotation()));
      path = new PathPlannerPath(waypoints, rotations, new ArrayList<>(), new ArrayList<>(), new ArrayList<>(), SwerveConstants.telePathConstraints, null, new GoalEndState(0, properPoints.get(maxIndex + 1).getRotation()), false);

      path.preventFlipping = true;

      SwerveSubsystem.field.getObject("path").setPoses(path.getPathPoses());
      
      followPath = AutoBuilder.followPath(path);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(path != null){
      CommandScheduler.getInstance().schedule(followPath);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
