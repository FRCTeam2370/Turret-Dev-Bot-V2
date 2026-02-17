// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.Formatter.BigDecimalLayoutForm;
import java.util.function.Supplier;

import org.ejml.dense.row.misc.DeterminantFromMinor_FDRM;
import org.photonvision.PhotonUtils;

// import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;
//import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.SwerveModule;
import frc.robot.Lib.Utils.BallLogic;

public class SwerveSubsystem extends SubsystemBase {
  private ObjectDetection mObjectDetection;
  public static Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
  private static Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
  public SwerveModule[] mSwerveModules;
  public static SwerveDriveOdometry odometry;

  public static PIDController rotationPIDauto = new PIDController(0.075, 0.0, 0.01);
  public static PIDController rotationPID = new PIDController(0.5, 0, 0);

  public static Field2d field = new Field2d();

  public static SwerveDrivePoseEstimator poseEstimator;

  public static Optional<Alliance> color;

  private static Transform2d RobotToTurret = TurretConstants.RobotToTurret;
  private static FieldObject2d turret = field.getObject("Turret");
  private static FieldObject2d HubFieldPose = field.getObject("HubPose");
  private static Pose2d HubPose = new Pose2d(0,0, new Rotation2d());
  private static FieldObject2d detectorCam = field.getObject("Detection Cam");
  private static FieldObject2d nextClosestBall = field.getObject("next closest ball");

  private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("MyPose", Pose2d.struct).publish();


  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(ObjectDetection mObjectDetection) {
    this.mObjectDetection = mObjectDetection;
    color = DriverStation.getAlliance();
    SmartDashboard.putString("Alliace Color", color.toString());

    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    configureGyro();

    mSwerveModules = new SwerveModule[] {
      new SwerveModule(0, Constants.FLConstants.FLConstants),
      new SwerveModule(1, Constants.FRConstants.FRConstants),
      new SwerveModule(2, Constants.BLConstants.BLConstants),
      new SwerveModule(3, Constants.BRConstants.BRConstants)
    };

    odometry = new SwerveDriveOdometry(Constants.SwerveConstants.kinematics, getRotation2d(), getModulePositions());
    poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.kinematics, getRotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), getRotation2d()));

    resetOdometry(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), readGyro()));

    configurePathPlanner();

    

    //Logging
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Robot Rotational Velocity", gyro.getAngularVelocityZWorld().getValueAsDouble());
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Mod 0 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[0].getCANcoder().getDegrees()).getDegrees());
    // SmartDashboard.putNumber("Mod 1 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[1].getCANcoder().getDegrees()).getDegrees());
    // SmartDashboard.putNumber("Mod 2 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[2].getCANcoder().getDegrees()).getDegrees());
    // SmartDashboard.putNumber("Mod 3 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[3].getCANcoder().getDegrees()).getDegrees());

    
    // SmartDashboard.putNumber("Wheel MPS", mSwerveModules[0].getWheelMPS());
    // SmartDashboard.putNumber("Wheel Meters", mSwerveModules[0].getModuleMeters());

    // SmartDashboard.putNumber("wheel Velocity", mSwerveModules[0].getWheelVelocity());
    // SmartDashboard.putNumber("drive Voltage", mSwerveModules[0].getWheelVoltage());

    SmartDashboard.putNumber("Gyro Val", getRotation2d().getDegrees());
    SmartDashboard.putNumber("gyro 0-360 val", getgyro0to360(0).getDegrees());
    // SmartDashboard.putNumber("Heading", getHeading());
    // SmartDashboard.putNumber("pose x", poseEstimator.getEstimatedPosition().getX());
    // SmartDashboard.putNumber("pose y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("pose rot", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    nextClosestBall.setPose(getClosestBall());

    // SmartDashboard.putNumber("Odometry x", odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Odometry y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Calculated Turret Angle", turretRotationToPose(FieldConstants.HubFieldPose).getDegrees());
    NetworkTableInstance.getDefault().getTable("fuelCV").getEntry("Camera Pose").setDoubleArray(new Double[]{detectionCamToField().getX(), detectionCamToField().getY(), getgyro0to360(180).getRadians()});

    updateOdometry();
    //odometry.update(getRotation2d(), getModulePositions());//USE THIS WHEN TESTING AUTOS WITHOUT FIELD LOCALIZATION
    resetOdometry(poseEstimator.getEstimatedPosition());

    field.setRobotPose(poseEstimator.getEstimatedPosition());
    turret.setPose(new Pose2d(turretToField().getTranslation(), turretRotationToPose(HubPose)));
    //HubFieldPose.setPose(HubPose);
    detectorCam.setPose(detectionCamToField());

    publisher.set(poseEstimator.getEstimatedPosition());
  }


  //This method calculates the position of the turret relative to the field 
  //It uses the position of the turret relative to the robot relative to the field
  public static Pose2d turretToField(){
    double xlocal = RobotToTurret.getX();
    double ylocal = RobotToTurret.getY();

    double xGlobal = xlocal * Math.cos(getRotation2d().getRadians()) + ylocal * Math.sin(getRotation2d().getRadians()) + poseEstimator.getEstimatedPosition().getX();
    double yGlobal = ylocal * Math.cos(getRotation2d().getRadians()) + xlocal * Math.sin(getRotation2d().getRadians()) + poseEstimator.getEstimatedPosition().getY();

    return new Pose2d(xGlobal, yGlobal, poseEstimator.getEstimatedPosition().getRotation());
  }

  public static Pose2d detectionCamToField(){
    double xlocal = VisionConstants.objectDetectionRobotToCamera.getX();
    double ylocal = VisionConstants.objectDetectionRobotToCamera.getY();

    double xGlobal = xlocal * Math.cos(getgyro0to360(0).getRadians()) + ylocal * Math.sin(getgyro0to360(0).getRadians()) + poseEstimator.getEstimatedPosition().getX();
    double yGlobal = ylocal * Math.cos(getgyro0to360(0).getRadians()) + xlocal * Math.sin(getgyro0to360(0).getRadians()) + poseEstimator.getEstimatedPosition().getY();

    return new Pose2d(xGlobal, yGlobal, getgyro0to360(180));
  }

  //This method calculates the angle from the turret to the target Pose2d
  public static Rotation2d turretRotationToPose(Pose2d pose){
    Pose2d turretpose = turretToField();//calculates the turrets pose relative to the field
    double thetaWorldToTarget = Math.atan2((turretpose.getY() - pose.getY()), (turretpose.getX() - pose.getX()));//calculates the angle from the turret's point to the target point
    double thetaTurretToTarget = thetaWorldToTarget + Math.PI + TurretConstants.TurretCableChainPoint.getRadians()//uses the thetaWorldToTarget and subtracts the robot's rotation to get the rotation from the turret (adding pi here is an offset)
     - getgyro0to360(0).getRadians() //subtracting the robot's rotation
     - (Math.toRadians(gyro.getAngularVelocityZWorld().getValueAsDouble()) * 0.02);//adding angular velocity lookahead
    thetaTurretToTarget = ((thetaTurretToTarget % (2*Math.PI)) + (2*Math.PI)) % (2*Math.PI);//Returns the thetaTurretToTarget value in the range of 0-360 degrees

    return Rotation2d.fromRadians(thetaTurretToTarget);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.kinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotationPID.calculate(rotation), readGyro()) :
      new ChassisSpeeds(translation.getX(), translation.getY(), rotationPIDauto.calculate(rotation))
    );    

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    mSwerveModules[0].setDesiredState(swerveModuleStates[0], isOpenLoop);
    mSwerveModules[1].setDesiredState(swerveModuleStates[1], isOpenLoop);
    mSwerveModules[2].setDesiredState(swerveModuleStates[2], isOpenLoop);
    mSwerveModules[3].setDesiredState(swerveModuleStates[3], isOpenLoop);
  }

  public static Rotation2d getgyro0to360(double offset){
    return Rotation2d.fromDegrees((((getRotation2d().getDegrees() + offset) % 360)+360)%360);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[]{
      mSwerveModules[0].getPosition(),
      mSwerveModules[1].getPosition(),
      mSwerveModules[2].getPosition(),
      mSwerveModules[3].getPosition()
    };
    return positions;
  }

  public static void configureGyro(){
    resetGyro();
  }
  
  public static void resetGyro(){
    gyro.setYaw(Constants.SwerveConstants.gyroOffset);
  }

  public static Rotation2d readGyro(){
    return color.isPresent() && color.get() == Alliance.Blue ? gyro.getRotation2d() : gyro.getRotation2d().rotateBy(Rotation2d.fromDegrees(180));
  }

  public static Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(readGyro().getDegrees() + 90);
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);//pose.getRotation()
    //poseEstimator.resetPose(pose);
  }

  public void updateOdometry(){
    poseEstimator.update(getRotation2d(), getModulePositions());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();//poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.SwerveConstants.kinematics.toChassisSpeeds(
      mSwerveModules[0].getState(),
      mSwerveModules[1].getState(),
      mSwerveModules[2].getState(),
      mSwerveModules[3].getState()
    );
  }

  public Command PathfindToPose(Supplier<Pose2d> poseSupplier){
    return new DeferredCommand(()-> AutoBuilder.pathfindToPose(poseSupplier.get(), Constants.SwerveConstants.telePathConstraints), Set.of(this));
  }

  public Command PathfindThenFollow(PathPlannerPath path){
    path.preventFlipping = false;
    return new DeferredCommand(()-> AutoBuilder.pathfindThenFollowPath(path, Constants.SwerveConstants.telePathConstraints), Set.of(this));
  }

  public Command followPath(PathPlannerPath path){
    return AutoBuilder.followPath(path);
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond / Constants.SwerveConstants.maxAngularVelocity, false, false);
  }

  
  public void configurePathPlanner(){
      RobotConfig config = new RobotConfig(36.287, 4.095, new ModuleConfig(0.0508, Constants.SwerveConstants.maxSpeed, 1.200, DCMotor.getKrakenX60(1), 60, 1), Constants.SwerveConstants.kinematics.getModules());
      try{
        config = RobotConfig.fromGUISettings();
        System.out.println("---------------------------Configured from GUI---------------------------");
      }catch(Exception exception){
        exception.printStackTrace();
        System.out.println("Doesn't like to config from gui settings");
      }
      AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveRobotRelative(speeds),//drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond / Constants.SwerveConstants.maxAngularVelocity, false, false),//drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond / 3.1154127, false, true),//driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(0, 0.0, 0.0), // Translation PID constants// 3.8 - p
                      new PIDConstants(0, 0.0, 0.0) // Rotation PID constants//kp 0.00755, ki 0.0001
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                if (color.isPresent()) {
                  return color.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[]{
      mSwerveModules[0].getState(),
      mSwerveModules[1].getState(),
      mSwerveModules[2].getState(),
      mSwerveModules[3].getState()
    };
    return states;
  }

  // public DeferredCommand driveThroughBalls(){
  //   //checks to make sure that there are points to pathfind through, if not it doesn't make a path
  //   while(mObjectDetection.ballPoses.size() > 0){
  //     //rearanges the random assortment of ball points to an organized and effiecent order based on which ball is next closest
  //     ArrayList<Pose2d> properPoints = BallLogic.rearangePoints(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), getgyro0to360(0)), mObjectDetection.ballPoses, 2);
  //     ArrayList<RotationTarget> rotations = new ArrayList<>();

  //     //gets the max distance of the entire path
  //     double maxDistance = 0;
  //     for(int i = 1; i < properPoints.size(); i++){
  //       maxDistance += PhotonUtils.getDistanceToPose(properPoints.get(i - 1), properPoints.get(i));
  //     }

  //     //adds rotation targets to the path so the intake will always face the point it is driving to
  //     for(int i = 1; i < properPoints.size(); i++){
  //       //gets the distance of the point from the start
  //       double distanceFromStart = PhotonUtils.getDistanceToPose(properPoints.get(0), properPoints.get(i));
  //       //adds the rotation target at the percentage of the path that the ball is at
  //       rotations.add(new RotationTarget(distanceFromStart/maxDistance, properPoints.get(i).getRotation()));
  //       SmartDashboard.putNumber("percentage of the path that the rotation target is at", distanceFromStart/maxDistance);
  //     }
  //     SmartDashboard.putNumber("Size of properpoints", properPoints.size());

  //     //creates a new list of waypoints for the path
  //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(properPoints);

  //     SmartDashboard.putNumber("size of waypoints", waypoints.size());

  //     //contructs a path out of the waypoints, rotations, tele path constraints, and a goal end state
  //     //path = new PathPlannerPath(waypoints, SwerveConstants.telePathConstraints, null, new GoalEndState(0, properPoints.get(maxIndex + 1).getRotation()));
  //     PathPlannerPath path = new PathPlannerPath(waypoints, rotations, new ArrayList<>(), new ArrayList<>(), new ArrayList<>(), SwerveConstants.telePathConstraints, null, new GoalEndState(0, properPoints.get(properPoints.size()-1).getRotation()), false);

  //     path.preventFlipping = true;//prevent flipping the path based on alliance color

  //     SwerveSubsystem.field.getObject("path").setPoses(path.getPathPoses());//log the path to the field2d
      
  //     SmartDashboard.putString("Failed To follow path", "didn't follow path");
  //     return new DeferredCommand(()-> AutoBuilder.pathfindToPose(properPoints.get(1),  SwerveConstants.telePathConstraints), Set.of(this, mObjectDetection));//create the command to follow the path
  //   }
  //   return new DeferredCommand(()-> new Command(){}, Set.of(this, mObjectDetection));
  // }

  private PathPlannerPath getPath(){
    PathPlannerPath path = null;
    List<Waypoint> waypoints = null;
    ArrayList<RotationTarget> rotations = new ArrayList<>();
    ArrayList<PointTowardsZone> pointtowards = new ArrayList<>();
    ArrayList<Pose2d> poses = new ArrayList<>();
    poses.add(SwerveSubsystem.poseEstimator.getEstimatedPosition());
    
    if(mObjectDetection.ballx.length > 0 && mObjectDetection.bally.length > 0){
      for(int i = 0; i< mObjectDetection.ballx.length; i++){
        poses.add(new Pose2d(mObjectDetection.ballx[i], mObjectDetection.bally[i], new Rotation2d()));
      }

      // for(int i = 1; i < poses.size(); i++){
      //   poses.set(i, new Pose2d(poses.get(i).getTranslation(), Rotation2d.fromDegrees(BallLogic.getRotation2dToPose(poses.get(i-1), poses.get(i)).getDegrees())));
      // }

      //adds rotation targets to the path so the intake will always face the point it is driving to
      for(int i = 1; i < poses.size() - 1; i++){
        //adds the rotation target at the percentage of the path that the ball is at
        rotations.add(new RotationTarget(i, Rotation2d.fromDegrees(BallLogic.getRotation2dToPose(poses.get(i-1), poses.get(i)).getDegrees() + VisionConstants.intakeSideRelativeToFront.getDegrees())));
      }
      
      // for(int i = 1; i<poses.size() - 1; i++){
      //   pointtowards.add(new PointTowardsZone("point towards ball" + i, poses.get(i).getTranslation(), i-1, i));
      // }

      waypoints = PathPlannerPath.waypointsFromPoses(poses);

      SmartDashboard.putNumber("size of waypoints", waypoints.size());
      
      path = new PathPlannerPath(waypoints, rotations, pointtowards, new ArrayList<>(), new ArrayList<>(), SwerveConstants.telePathConstraints, null, new GoalEndState(0, Rotation2d.fromDegrees(BallLogic.getRotation2dToPose(poses.get(poses.size()-2), poses.get(poses.size() -1)).getDegrees() + VisionConstants.intakeSideRelativeToFront.getDegrees())), false);

      path.preventFlipping = true;

      SwerveSubsystem.field.getObject("path").setPoses(path.getPathPoses());
    }
    return path;
  }
  
  public DeferredCommand driveThroughBalls(){
    //checks to make sure that there are points to pathfind through, if not it doesn't make a path
    try{
      return new DeferredCommand(()-> AutoBuilder.followPath(getPath()), Set.of(this, mObjectDetection));//create the command to follow the path
    }catch(Exception e){
      System.out.println(e);
    }
    return null;
  }

  public Pose2d getClosestBall(){
    if(mObjectDetection.ballx != null && mObjectDetection.ballx.length > 0 && mObjectDetection.bally.length > 0){
      Pose2d ballPose = new Pose2d(mObjectDetection.ballx[0], mObjectDetection.bally[0], new Rotation2d());
      return new Pose2d(ballPose.getTranslation(), Rotation2d.fromDegrees(BallLogic.getRotation2dToPose(poseEstimator.getEstimatedPosition(), ballPose).getDegrees() + VisionConstants.intakeSideRelativeToFront.getDegrees()));
    }else{
      return poseEstimator.getEstimatedPosition();
    }
    
  }

  public DeferredCommand driveToClosestBall(Supplier<Pose2d> pose){
    try{
      return new DeferredCommand(()-> AutoBuilder.pathfindToPose(pose.get(), SwerveConstants.telePathConstraints), Set.of(this, mObjectDetection));
    }catch (Exception e){
      System.out.println(e);
    }
    return null;
  }
}
