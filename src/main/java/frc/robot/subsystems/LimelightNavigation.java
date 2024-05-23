// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
//import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanID;
import frc.robot.LimelightHelpers;

public class LimelightNavigation extends SubsystemBase {
  Translation2d m_FrontLeftWheel_Position;
  Translation2d m_FrontRightWheel_Position;
  Translation2d m_RearLeftWheel_Position;
  Translation2d m_RearRightWheel_Position;

  RelativeEncoder m_FrontLeftWheel_Endocer;
  RelativeEncoder m_FrontRightWheel_Encoder;
  RelativeEncoder m_RearLeftWheel_Encoder;
  RelativeEncoder m_RearRightWheel_Encoder;

  MecanumDriveKinematics m_Kinematics;
  // MecanumDriveOdometry m_DriveOdometry;
  static MecanumDrivePoseEstimator m_DrivePoseEstimator;

  private final Pigeon2 m_pigeon2;
  boolean m_InitializeDFromTag = false;

  double m_latency = 0.0;
  double m_limelightSamplesCaptured = 0;

  //private Pigeon2Configuration pigeon2Config;

  int x;

  /** Creates a new LimelightNavigation. */
  public LimelightNavigation(Map<String, RelativeEncoder> encoders) {
    // BUGBUG: Get this in RobotContainer, pass to this subsystem.
    Map<String, RelativeEncoder> Encoders = encoders;
    m_FrontLeftWheel_Endocer = Encoders.get("Front Left");
    m_FrontRightWheel_Encoder = Encoders.get("Front Right");
    m_RearLeftWheel_Encoder = Encoders.get("Rear Left");
    m_RearRightWheel_Encoder = Encoders.get("Rear Right");


    Pose2d initialPoseMeters = new Pose2d();
    m_Kinematics = new MecanumDriveKinematics(Constants.Misc.FrontLeftDriveWheel_Position_Meters,
        Constants.Misc.FrontRightDriveWheel_Position_Meters,
        Constants.Misc.RearLeftDriveWheel_Position_Meters, Constants.Misc.RearRightDriveWheel_Position_Meters);
    m_DrivePoseEstimator = new MecanumDrivePoseEstimator(m_Kinematics, new Rotation2d(0),
        new MecanumDriveWheelPositions(
            m_FrontLeftWheel_Endocer.getPosition()*Constants.Misc.metersPerTick, 
            m_FrontRightWheel_Encoder.getPosition()*Constants.Misc.metersPerTick,
            m_RearLeftWheel_Encoder.getPosition()*Constants.Misc.metersPerTick,
             m_RearRightWheel_Encoder.getPosition()*Constants.Misc.metersPerTick),
        initialPoseMeters);

    m_pigeon2 = new Pigeon2(CanID.Pigeon2);
    var pigeon2Config = new Pigeon2Configuration();
  }

  public Rotation2d getPoseHeading() {
    return m_DrivePoseEstimator.getEstimatedPosition().getRotation();
  }

  public Rotation2d getGyroHeading() {
    return m_pigeon2.getRotation2d();
  }

  public double getRoll() {
    return m_pigeon2.getRoll().getValueAsDouble();
  }

  public void ledControls(int ControlValue) {
    if (ControlValue == 1) {
      ControlValue = 2;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ControlValue);    
    } else {
      ControlValue = 1;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ControlValue);    
    }
  }

  public void resetFieldOrient() {
    m_pigeon2.reset();
    System.out.println("resetfieldorient");
  }

  public void resetPosition() {
    m_InitializeDFromTag = false;
    //m_pigeon2.reset();
    // This method will be called once per scheduler run
    if (LimelightHelpers.getTV(Constants.Misc.LimelightName) == true) {
      Pose2d robotPose2d = LimelightHelpers.getBotPose2d_wpiBlue(Constants.Misc.LimelightName);
      var MecanumDriveWheelPositions = new MecanumDriveWheelPositions(
          m_FrontLeftWheel_Endocer.getPosition()*Constants.Misc.metersPerTick, 
          m_FrontRightWheel_Encoder.getPosition()*Constants.Misc.metersPerTick,
          m_RearLeftWheel_Encoder.getPosition()*Constants.Misc.metersPerTick, 
          m_RearRightWheel_Encoder.getPosition()*Constants.Misc.metersPerTick);
      m_DrivePoseEstimator.resetPosition(m_pigeon2.getRotation2d(), MecanumDriveWheelPositions, robotPose2d);
      m_InitializeDFromTag = true;
    }
  }

  @Override
  public void periodic() {
    double cl;
    double tl;
    if (m_InitializeDFromTag == false) {
     resetPosition();
    } else {
      MecanumDriveWheelPositions positions = new MecanumDriveWheelPositions(
        m_FrontLeftWheel_Endocer.getPosition()*Constants.Misc.metersPerTick,
          m_FrontRightWheel_Encoder.getPosition()*Constants.Misc.metersPerTick,
          m_RearLeftWheel_Encoder.getPosition()*Constants.Misc.metersPerTick,
          m_RearRightWheel_Encoder.getPosition()*Constants.Misc.metersPerTick);
      m_DrivePoseEstimator.update(m_pigeon2.getRotation2d(), positions);

      if (LimelightHelpers.getTV(Constants.Misc.LimelightName) == true) {
        Pose2d robotPose2d = LimelightHelpers.getBotPose2d_wpiBlue(Constants.Misc.LimelightName);
        cl = LimelightHelpers.getLatency_Capture("limelight");
        tl = LimelightHelpers.getLatency_Pipeline("limelight");
        m_limelightSamplesCaptured = m_limelightSamplesCaptured + 1;
        m_latency = Timer.getFPGATimestamp()- tl /1000 - cl /1000;
        m_DrivePoseEstimator.addVisionMeasurement(robotPose2d, m_latency);
      }
    }
    Pose2d pose = m_DrivePoseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("Robot Latency (Milliseconds)", m_latency);
    SmartDashboard.putNumber("Limelight Samples Captured", m_limelightSamplesCaptured);
    SmartDashboard.putNumber("Robot X", pose.getX());
    SmartDashboard.putNumber("Robot Y", pose.getY());
    SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("updated heading", m_pigeon2.getRotation2d().getDegrees());

    SmartDashboard.putNumber("Robot Pitch", m_pigeon2.getPitch().getValueAsDouble());
    SmartDashboard.putNumber("Robot Yaw", m_pigeon2.getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Robot Roll", m_pigeon2.getRoll().getValueAsDouble());
    double currentHeading = SmartDashboard.getNumber("updated heading", m_pigeon2.getRotation2d().getDegrees());
  }

  public Pose2d getPose2d() {
    return m_DrivePoseEstimator.getEstimatedPosition();
  }

  public static void resetgyro () {
  }
}

// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html.