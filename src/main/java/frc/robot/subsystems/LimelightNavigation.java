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
  MecanumDrivePoseEstimator m_DrivePoseEstimator;

  public static Pigeon2 m_pigeon2;
  boolean m_InitializeDFromTag = false;


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
            m_FrontLeftWheel_Endocer.getPosition(), m_FrontRightWheel_Encoder.getPosition(),
            m_RearLeftWheel_Encoder.getPosition(), m_RearRightWheel_Encoder.getPosition()),
        initialPoseMeters);

    m_pigeon2 = new Pigeon2(CanID.Pigeon2);
    var pigeon2Config = new Pigeon2Configuration();
  }

  public static Rotation2d getHeading() {
    return m_pigeon2.getRotation2d();
  }

  public void ledsOff() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void ledsOn() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
  }

  public void resetFieldOrient() {
    m_pigeon2.reset();
    System.out.println("resetfieldorient");
  }

  public void resetPosition() {
    //m_pigeon2.reset();
    // This method will be called once per scheduler run
    if (LimelightHelpers.getTV(Constants.Misc.LimelightName) == true) {
      Pose2d robotPose2d = LimelightHelpers.getBotPose2d_wpiRed(Constants.Misc.LimelightName);
      var MecanumDriveWheelPositions = new MecanumDriveWheelPositions(
          m_FrontLeftWheel_Endocer.getPosition(), m_FrontRightWheel_Encoder.getPosition(),
          m_RearLeftWheel_Encoder.getPosition(), m_RearRightWheel_Encoder.getPosition());

      m_DrivePoseEstimator.resetPosition(m_pigeon2.getRotation2d(), MecanumDriveWheelPositions, robotPose2d);
      m_InitializeDFromTag = true;
    }
  }

  @Override
  public void periodic() {
    if (m_InitializeDFromTag == false) {
     resetPosition();
    } else {
      MecanumDriveWheelPositions positions = new MecanumDriveWheelPositions(m_FrontLeftWheel_Endocer.getPosition(),
          m_FrontRightWheel_Encoder.getPosition(), m_RearLeftWheel_Encoder.getPosition(),
          m_RearRightWheel_Encoder.getPosition());
      m_DrivePoseEstimator.update(m_pigeon2.getRotation2d(), positions);

      if (LimelightHelpers.getTV(Constants.Misc.LimelightName) == true) {
        Pose2d robotPose2d = LimelightHelpers.getBotPose2d_wpiRed(Constants.Misc.LimelightName);
        m_DrivePoseEstimator.addVisionMeasurement(robotPose2d, Timer.getFPGATimestamp());
      }
    }
    Pose2d pose = m_DrivePoseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("Robot X", pose.getX());
    SmartDashboard.putNumber("Robot Y", pose.getY());
    SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("updated heading", m_pigeon2.getRotation2d().getDegrees());

    SmartDashboard.putNumber("Robot Pitch", m_pigeon2.getPitch().getValueAsDouble());
    SmartDashboard.putNumber("Robot Yaw", m_pigeon2.getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Robot Roll", m_pigeon2.getRoll().getValueAsDouble());
    double currentHeading = SmartDashboard.getNumber("updated heading", m_pigeon2.getRotation2d().getDegrees());
    resetPosition();
  }

  public Pose2d getPose2d() {
    return m_DrivePoseEstimator.getEstimatedPosition();
  }

  public static void resetgyro () {
  //  m_pigeon2.reset();
  }
}

// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html.