// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climberSubsystem extends SubsystemBase {
  Pigeon2 m_pigeon2;

  private final LeftClimberSubsystem m_leftClimber = new LeftClimberSubsystem();
  private final RightClimberSubsystem m_rightClimber = new RightClimberSubsystem();

  /** Creates a new climberSubsystem. */
  public climberSubsystem(){
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

  private void raiseLower(double cmd){
    double error = m_pigeon2.getRoll().getValueAsDouble();
    double levelCommand = MecanumDriveSubsystem.deadzone(error, Constants.Misc.climberDeadZone)*Constants.Misc.climberP;
    double leftSpeed = cmd-levelCommand;
    double rightSpeed = cmd+levelCommand;
    m_leftClimber.motor(leftSpeed);
    m_rightClimber.motor(rightSpeed);
  }

  public void stop() {
   double leftSpeed = 0;
   double rightSpeed = 0;
   m_leftClimber.motor(leftSpeed);
   m_rightClimber.motor(rightSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}