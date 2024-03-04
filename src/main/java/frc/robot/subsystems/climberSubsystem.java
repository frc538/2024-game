// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climberSubsystem extends SubsystemBase {
  CANSparkMax climberLeft;
  CANSparkMax climberRight;
  Pigeon2 m_pigeon2;

  /** Creates a new climberSubsystem. */
  public climberSubsystem() {
    climberLeft = new CANSparkMax(Constants.CANIDs.leftClimber, MotorType.kBrushless);
    climberRight = new CANSparkMax(Constants.CANIDs.rightClimber, MotorType.kBrushless);
    
    m_pigeon2 = LimelightNavigation.m_pigeon2;

    climberLeft.restoreFactoryDefaults();
    climberLeft.setInverted(false);
    climberLeft.burnFlash();

    climberRight.restoreFactoryDefaults();
    climberRight.setInverted(true);
    climberRight.burnFlash();
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
    climberLeft.set(cmd-levelCommand);
    climberRight.set(cmd+levelCommand);
  }

  public void stop() {
    climberLeft.set(0);
    climberRight.set(-0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}