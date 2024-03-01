// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  CANSparkMax leftClimber;
  CANSparkMax rightClimber;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    leftClimber = new CANSparkMax(Constants.CANIDs.leftClimber, MotorType.kBrushless);
    rightClimber = new CANSparkMax(Constants.CANIDs.rightClimber, MotorType.kBrushless);

    leftClimber.restoreFactoryDefaults();
    leftClimber.setInverted(false);
    leftClimber.burnFlash();

    rightClimber.restoreFactoryDefaults();
    rightClimber.setInverted(true);
    rightClimber.burnFlash();
  }

  public void leftRaise() {
    leftClimber.set(1);
    rightClimber.set(0);
  }

  public void leftLower() {
    leftClimber.set(-1);
    rightClimber.set(0);
  }

  public void rightRaise() {
    rightClimber.set(1);
    leftClimber.set(0);
  }

  public void rightLower() {
    rightClimber.set(-1);
    leftClimber.set(0);
  }

  public void bothRaise() {
    rightClimber.set(1);
    leftClimber.set(1);
  }

  public void bothLower() {
    rightClimber.set(-1);
    leftClimber.set(-1);
  }

  public void bothStop() {
    rightClimber.set(0);
    leftClimber.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
