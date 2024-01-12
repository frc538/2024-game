// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDriveSubsystem extends SubsystemBase {
  CANSparkMax frontLeft;
  CANSparkMax frontRight;
  CANSparkMax rearLeft;
  CANSparkMax rearRight;

  MecanumDrive driveBase;

  /** Creates a new MechaniumDrive. */
  public MecanumDriveSubsystem() {
    frontLeft = new CANSparkMax(Constants.CANSparkMaxID.FrontLeftDriveMotor, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.CANSparkMaxID.FrontRightDriveMotor, MotorType.kBrushless);
    rearLeft = new CANSparkMax(Constants.CANSparkMaxID.RearLeftDriveMotor, MotorType.kBrushless);
    rearRight = new CANSparkMax(Constants.CANSparkMaxID.RearRightriveMotor, MotorType.kBrushless);

    frontLeft.restoreFactoryDefaults();
    frontLeft.setInverted(false);
    frontLeft.burnFlash();

    rearLeft.restoreFactoryDefaults();
    rearLeft.setInverted(false);
    rearLeft.burnFlash();

    frontRight.restoreFactoryDefaults();
    frontRight.setInverted(true);
    frontRight.burnFlash();

    rearRight.restoreFactoryDefaults();
    rearRight.setInverted(true);
    rearRight.burnFlash();

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