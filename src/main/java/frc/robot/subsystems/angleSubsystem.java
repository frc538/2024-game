// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class angleSubsystem extends SubsystemBase {
  /** Creates a new angleSubsystem. */
  CANSparkMax angleTop;
  CANSparkMax angleBottom;
  public angleSubsystem() {
    angleTop = new CANSparkMax(Constants.CANIDs.sliderA, MotorType.kBrushless);
    angleBottom = new CANSparkMax(Constants.CANIDs.sliderB, MotorType.kBrushless);

    angleTop.restoreFactoryDefaults();
    angleTop.setIdleMode(IdleMode.kBrake);
    angleTop.setInverted(false);
    angleTop.burnFlash();

    angleBottom.restoreFactoryDefaults();
    angleBottom.setIdleMode(IdleMode.kBrake);
    angleBottom.setInverted(false);
    angleBottom.burnFlash();

    angleBottom.follow(angleTop);
  }

  public void angleUp(){
    angleTop.set(Constants.Misc.angleUpSpeed);
  }

  public void angleDown(){
    angleTop.set(Constants.Misc.angleDownSpeed);
  }

  public void stop(){
    angleTop.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
