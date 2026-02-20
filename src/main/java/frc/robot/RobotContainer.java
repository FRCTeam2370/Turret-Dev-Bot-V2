// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Lib.Utils.TurretLogic;
import frc.robot.Commands.FindDriveKS;
import frc.robot.Commands.PathfindThroughBalls;
import frc.robot.Commands.ResetGyro;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.IntakeCommands.RunIntakeForPercentSpeed;
import frc.robot.Commands.TurretCommands.PointTurretAtPoint;
import frc.robot.Subsystems.FieldInfo;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ObjectDetection;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.Vision;

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);

  
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final TurretSubsystem mTurretSubsystem = new TurretSubsystem();
  private final Vision mVision = new Vision();
  private final ObjectDetection mObjectDetection = new ObjectDetection();
  private final FieldInfo mFieldInfo = new FieldInfo();
  private final SwerveSubsystem mSwerve = new SwerveSubsystem(mObjectDetection);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //Put all NamedCommands here
    NamedCommands.registerCommand("Test", new ResetGyro(mSwerve));

    configureBindings();
  }

  private void configureBindings() {
    mSwerve.setDefaultCommand(new TeleopSwerve(mSwerve, ()-> driver.getRawAxis(0), ()-> -driver.getRawAxis(1), ()-> driver.getRawAxis(4), ()-> false));

    //driver.b().whileTrue(mSwerve.PathfindToPose(() -> new Pose2d(16.3,0.49, Rotation2d.fromDegrees(-50))));
    // try {
    //   driver.b().whileTrue(mSwerve.PathfindThenFollow(PathPlannerPath.fromPathFile("CoralLoadLR")));
    // } catch (FileVersionException | IOException | ParseException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    // driver.x().whileTrue(mSwerve.PathfindToPose(() -> Constants.RedSidePoses.REDFRONTLEFTSCORE));
    // driver.y().whileTrue(mSwerve.PathfindToPose(()-> Constants.RedSidePoses.REDBACKLEFTSCORE));

    driver.a().onTrue(new ResetGyro(mSwerve));

    driver.leftTrigger().toggleOnTrue(new FindDriveKS(mSwerve));

    driver.rightBumper().toggleOnTrue(new RunIntakeForPercentSpeed(70, mIntakeSubsystem));
    driver.leftBumper().whileTrue(new RunIntakeForPercentSpeed(-20, mIntakeSubsystem));

    driver.b().toggleOnTrue(new PointTurretAtPoint(FieldConstants.HubFieldPose, mTurretSubsystem, mSwerve));
    driver.y().toggleOnTrue(new PointTurretAtPoint(FieldConstants.AimPose1, mTurretSubsystem, mSwerve));
    driver.x().toggleOnTrue(new PointTurretAtPoint(FieldConstants.AimPose2, mTurretSubsystem, mSwerve));

    driver.rightTrigger().whileTrue(mSwerve.driveThroughBalls());
    driver.povUp().whileTrue(mSwerve.driveToClosestBall(()-> mSwerve.getClosestBall()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
