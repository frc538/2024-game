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

import edu.wpi.first.math.geometry.Pose2d;
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
  private double driveGain = 0.1;
  
  LimelightNavigation m_LimelightNavigation;
  MecanumDrive driveBase;


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
    
  }
  public void setLimeLightNavigation (LimelightNavigation lNavigation) {
    m_LimelightNavigation = lNavigation;
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

  public double deadzone(double value, double dz) {
    if (value > dz) {
      return value - dz;
    }
    if (value < -dz) {
      return value + dz;
    }
    else {
      return 0;
    }
  }

  public void drive(double forwardSpeed, double rightSpeed, double rotatinalSpeed, double sliderValue) {
    double driveGain = 0.45*sliderValue+.55;
    driveBase.driveCartesian(
      deadzone(forwardSpeed,Constants.Misc.driveDeadzone)*driveGain, 
      deadzone(rightSpeed,Constants.Misc.driveDeadzone)*driveGain,
      deadzone(rotatinalSpeed,Constants.Misc.driveDeadzone)*driveGain);

  }

  public void alignRedAmp() {

    double desiredx = 16.52;
    double desiredy = 1.40;
    double desiredHeading = 1;

    alignTarget(desiredx, desiredy, desiredHeading);
  }

  public void alignBlueAmp() {

    double desiredx = 16.52;
    double desiredy = 1.40;
    double desiredHeading = 1;

    alignTarget(desiredx, desiredy, desiredHeading);
  }

  public void alignBlueSpeaker() {

    double desiredx = 16.52;
    double desiredy = 1.40;
    double desiredHeading = 1;

    alignTarget(desiredx, desiredy, desiredHeading);
  }

   public void alignRedSpeaker() {

    double desiredx = 16.52;
    double desiredy = 1.40;
    double desiredHeading = 1;

    alignTarget(desiredx, desiredy, desiredHeading);
  }

   public void alignRedSource() {

    double desiredx = 16.52;
    double desiredy = 1.40;
    double desiredHeading = 1;

    alignTarget(desiredx, desiredy, desiredHeading);
  }

   public void alignBlueSource() {

    double desiredx = 16.52;
    double desiredy = 1.40;
    double desiredHeading = 1;

    alignTarget(desiredx, desiredy, desiredHeading);
  }

  public void alignTarget(double desiredx, double desiredy, double desiredHeading) {
    double left = 0;
    double forward = 0;
    double rotationSpeed = 0;
    Pose2d pose = m_LimelightNavigation.getPose2d();
    double ydisplacement = pose.getY()-desiredy;
    double xdisplacement = pose.getX()-desiredx;
    double headingdisplacement = pose.getRotation().getDegrees()-desiredHeading;
    double kpHeading = 0.05;
    if (Double.isNaN(xdisplacement)) {
      left = 0;
    } else if(Math.abs(xdisplacement)<0.06) {
      left = 0;
    } else if(xdisplacement<0) {
      left = 0.1;
    }else {
      left = -0.1;
    }

    if (Double.isNaN(ydisplacement)) {
      forward = 0;
    } else if(Math.abs(ydisplacement)<0.06) {
      forward = 0;
    } else if(ydisplacement<0) {
      forward = 0.1;
    } else {
      forward = -0.1;
    }

    if (Double.isNaN(headingdisplacement)) {
      rotationSpeed = 0;
    } else {
      double error = headingdisplacement;
      if (error > 180.0) {
        error = -(360-error);
      }
      rotationSpeed = kpHeading*error;
    }

    drive(forward, left, rotationSpeed, 0);
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

  }
}
