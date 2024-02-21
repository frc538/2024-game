// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDriveSubsystem extends SubsystemBase {
  CANSparkMax frontLeft;
  CANSparkMax frontRight;
  CANSparkMax rearLeft;
  CANSparkMax rearRight;

  MecanumDrive driveBase;
  int x;
  int y;

  /** Creates a new MechaniumDrive. */
  public MecanumDriveSubsystem() {
    frontLeft = new CANSparkMax(Constants.CANIDs.FrontLeftDriveMotor, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.CANIDs.FrontRightDriveMotor, MotorType.kBrushless);
    rearLeft = new CANSparkMax(Constants.CANIDs.RearLeftDriveMotor, MotorType.kBrushless);
    rearRight = new CANSparkMax(Constants.CANIDs.RearRighDriveMotor, MotorType.kBrushless);


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

    if (RobotBase.isSimulation()) {

    REVPhysicsSim.getInstance().addSparkMax(frontLeft, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(frontRight, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(rearLeft, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(rearRight, DCMotor.getNEO(1));

    }

     x=0;
    
  }

  public Map<String,RelativeEncoder> GetEncoders()
  {
    Map<String, RelativeEncoder> Encoders = new HashMap<String,RelativeEncoder>();
    Encoders.put("Front Left", frontLeft.getEncoder());
    Encoders.put("Front Right", frontRight.getEncoder());
    Encoders.put("Rear Left", rearLeft.getEncoder());
    Encoders.put("Rear Right", rearRight.getEncoder());

    return Encoders;
  }

  public void drive(double forwardSpeed, double rightSpeed, double rotatinalSpeed) {
    driveBase.driveCartesian(forwardSpeed, rightSpeed, rotatinalSpeed);

    SmartDashboard.putNumber("y", y);
    y++;
  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run
    double frontLeftSpeed = frontLeft.getEncoder().getVelocity();
    double frontRightSpeed = frontRight.getEncoder().getVelocity();
    double rearLeftSpeed = rearLeft.getEncoder().getVelocity();
    double rearRightSpeed = rearRight.getEncoder().getVelocity();


    SmartDashboard.putNumber("frontLeftSpeed", frontLeftSpeed);
    SmartDashboard.putNumber("frontRightSpeed", frontRightSpeed);
    SmartDashboard.putNumber("rearLeftSpeed", rearLeftSpeed);
    SmartDashboard.putNumber("rearRightSpeed", rearRightSpeed);

    SmartDashboard.putNumber("X", x);
    x++;
  }
}
