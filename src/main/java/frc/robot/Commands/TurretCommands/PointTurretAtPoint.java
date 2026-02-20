// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TurretCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointTurretAtPoint extends Command {
  private Translation3d pose;
  private SwerveSubsystem mSwerve;
  /** Creates a new PointTurretAtPoint. */
  public PointTurretAtPoint(Translation3d pose, TurretSubsystem mTurretSubsystem, SwerveSubsystem mSwerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pose = pose;
    this.mSwerve = mSwerve;
    addRequirements(mTurretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TurretSubsystem.aimTurretAtPoint(mSwerve.getTurretPointTowardsPose(pose));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TurretSubsystem.aimTurretAtDegree(180);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
