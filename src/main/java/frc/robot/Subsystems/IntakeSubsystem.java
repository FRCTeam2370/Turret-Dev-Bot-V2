// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  public static TalonFX intakeRollers = new TalonFX(4);
  private static TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
  private static DigitalInput hallEffect = new DigitalInput(0);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    configIntake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("hall Effect Val", hallEffect.get());
  }

  public static void runIntake(double speed){
    intakeRollers.set(speed / 100);
  }

  private static void configIntake(){
    intakeRollers.setNeutralMode(NeutralModeValue.Coast);
    
    rollersConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollersConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;

    intakeRollers.getConfigurator().apply(rollersConfig);
  }
}
