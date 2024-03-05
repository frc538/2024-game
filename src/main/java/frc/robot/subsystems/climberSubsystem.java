// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climberSubsystem extends SubsystemBase {
  LeftClimberSubsystem m_lcs;
  RightClimberSubsystem m_rcs;
  Pigeon2 m_pigeon2;

  /** Creates a new climberSubsystem. */
  public climberSubsystem(LeftClimberSubsystem lcs, RightClimberSubsystem rcs) {
    m_lcs = lcs;
    m_rcs = rcs;
    m_pigeon2 = LimelightNavigation.m_pigeon2;

  }

  public void raise() {
    double raiseCommand = .8;
    raiseLower(raiseCommand);
  }

  public void lower() {
    double lowerCommand = -.8;
    raiseLower(lowerCommand);
  }

  private void raiseLower(double cmd) {
    double error = m_pigeon2.getRoll().getValueAsDouble();
    double levelCommand = MecanumDriveSubsystem.deadzone(error, Constants.Misc.climberDeadZone)
        * Constants.Misc.climberP;
    m_lcs.setSpeed(cmd - levelCommand);
    m_rcs.setSpeed(cmd + levelCommand);
  }

  public void stop() {
    m_lcs.stop();
    m_rcs.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}