// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MechaniumDrive extends SubsystemBase {
  public PWMSparkMax  FrontLeft;
  public PWMSparkMax  FrontRight;
  public PWMSparkMax  RearLeft;
  public PWMSparkMax  RearRight;

  /** Creates a new MechaniumDrive. */
  public MechaniumDrive() {
    FrontLeft   = new PWMSparkMax(Constants.PWM.FrontLeftDriveMotor);
    FrontRight  = new PWMSparkMax(Constants.PWM.FrontRightDriveMotor);
    RearLeft    = new PWMSparkMax(Constants.PWM.RearLeftDriveMotor);
    RearRight   = new PWMSparkMax(Constants.PWM.RearRightriveMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
