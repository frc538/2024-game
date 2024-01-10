// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDriveSubsystem extends SubsystemBase {
  PWMSparkMax frontLeft;
  PWMSparkMax frontRight;
  PWMSparkMax rearLeft;
  PWMSparkMax rearRight;

  MecanumDrive driveBase;

  /** Creates a new MechaniumDrive. */
  public MecanumDriveSubsystem() {
    frontLeft = new PWMSparkMax(Constants.PWM.FrontLeftDriveMotor);
    frontRight = new PWMSparkMax(Constants.PWM.FrontRightDriveMotor);
    rearLeft = new PWMSparkMax(Constants.PWM.RearLeftDriveMotor);
    rearRight = new PWMSparkMax(Constants.PWM.RearRightriveMotor);

    frontLeft.setInverted(false);
    rearLeft.setInverted(false);
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    driveBase = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  }

  public void drive(double forwardSpeed, double rightSpeed, double rotatinalSpeed) {
    driveBase.driveCartesian(forwardSpeed, rightSpeed, rotatinalSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
