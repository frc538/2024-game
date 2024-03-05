// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LeftClimberSubsystem extends SubsystemBase {
  CANSparkMax climber;

  public static double climberDeadzone(double value, double dz) {
    if (value > dz) {
      return value - dz;
    }
    if (value < -dz) {
      return value + dz;
    }
    else {
      return 0;
    }
  }

  /** Creates a new ClimberSubsystem. */
  public LeftClimberSubsystem() { 
    
    climber = new CANSparkMax(Constants.CANIDs.leftClimber, MotorType.kBrushless);

    climber.restoreFactoryDefaults();
    climber.setInverted(false);
    climber.burnFlash();
  }

  public void setSpeed(double speed) {
    climber.set(speed);
  }

  public void raise() {
    climber.set(1);
    }

  public void lower() {
    climber.set(-1);
  }

  public void stop() {
    climber.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
