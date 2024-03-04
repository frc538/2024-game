// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanID;

public class climberSubsystem extends SubsystemBase {
  CANSparkMax climberA;
  CANSparkMax climberB;
  /** Creates a new climberSubsystem. */
  public climberSubsystem() {
    climberA = new CANSparkMax(Constants.CANIDs.leftClimber, MotorType.kBrushless);
    climberB = new CANSparkMax(Constants.CANIDs.rightClimber, MotorType.kBrushless);
    
    m_pigeon2 = new Pigeon2(CanID.Pigeon2);

    climberA.restoreFactoryDefaults();
    climberA.setInverted(false);
    climberA.burnFlash();

    climberB.restoreFactoryDefaults();
    climberB.setInverted(true);
    climberB.burnFlash();
  }

  private Pigeon2 m_pigeon2;
  boolean m_InitializeDFromTag = false;
  // private Pigeon2Configuration pigeon2Config;

  public void raise() {}

  public void lower() {}

  public void stop() {
    climberA.set(0);
    climberB.set(-0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double roll = m_pigeon2.getRoll().getValueAsDouble();
    SmartDashboard.putNumber("roll", roll);
  }
}
