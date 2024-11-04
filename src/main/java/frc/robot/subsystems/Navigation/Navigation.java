// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Navigation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.DriveIO.DriveIOInputs;

public class Navigation extends SubsystemBase {

  private final NavigationIO io;
  public final NavigationIOInputsAutoLogged inputs = new NavigationIOInputsAutoLogged();

  private DriveIOInputs driveInputs;

  public MecanumDriveKinematics m_Kinematics;
  static MecanumDrivePoseEstimator m_DrivePoseEstimator;

  boolean m_InitializeDFromTag = false;

  double m_latency = 0.0;
  double m_limelightSamplesCaptured = 0;

  boolean flashBangOnOff = true;
  int x;

  StructPublisher<Pose2d> posePublisher;

  /** Creates a new LimelightNavigation. */
  public Navigation(NavigationIO io, DriveIOInputs dio) {
    this.io = io;

    driveInputs = dio;

    Pose2d initialPoseMeters = new Pose2d();
    m_Kinematics = new MecanumDriveKinematics(Constants.Misc.FrontLeftDriveWheel_Position_Meters,
        Constants.Misc.FrontRightDriveWheel_Position_Meters,
        Constants.Misc.RearLeftDriveWheel_Position_Meters, Constants.Misc.RearRightDriveWheel_Position_Meters);
    m_DrivePoseEstimator = new MecanumDrivePoseEstimator(m_Kinematics, new Rotation2d(0),
        new MecanumDriveWheelPositions(
            driveInputs.frontLeftRad * Constants.Misc.metersPerMotorRad,
            driveInputs.frontRightRad * Constants.Misc.metersPerMotorRad,
            driveInputs.rearLeftRad * Constants.Misc.metersPerMotorRad,
            driveInputs.rearRightRad * Constants.Misc.metersPerMotorRad),
        initialPoseMeters);

    //m_pigeon2 = new Pigeon2(CanID.Pigeon2);
    //var pigeon2Config = new Pigeon2Configuration();

    posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose2d", Pose2d.struct).publish();

  }

  public Rotation2d getPoseHeading() {
    return m_DrivePoseEstimator.getEstimatedPosition().getRotation();
  }

  public Rotation2d getGyroHeading() {
    return io.getRotation2d();
  }

  public double getRoll() {
    return inputs.gyroRoll;
  }

  public void toggleLEDS() {
    flashBangOnOff = !flashBangOnOff;
    ledControls();

  }

  public void ledControls() {
    int ControlValue = flashBangOnOff ? 1 : 3;
    if (flashBangOnOff) {
      if (ControlValue == 1) {
        ControlValue = 3;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ControlValue);
      } else {
        ControlValue = 1;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ControlValue);
      }
    } else {
      ControlValue = 1;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ControlValue);
    }
  }

  public void resetFieldOrient() {
    io.resetGyro();
    System.out.println("resetfieldorient");
  }

  public void resetPosition() {
    m_InitializeDFromTag = false;
    //m_pigeon2.reset();
    // This method will be called once per scheduler run
    if (LimelightHelpers.getTV(Constants.Misc.LimelightName) == true) {
      Pose2d robotPose2d = LimelightHelpers.getBotPose2d_wpiBlue(Constants.Misc.LimelightName);
      var MecanumDriveWheelPositions = new MecanumDriveWheelPositions(
            driveInputs.frontLeftRad*Constants.Misc.metersPerMotorRad, 
            driveInputs.frontRightRad*Constants.Misc.metersPerMotorRad,
            driveInputs.rearLeftRad*Constants.Misc.metersPerMotorRad,
            driveInputs.rearRightRad*Constants.Misc.metersPerMotorRad);
      m_DrivePoseEstimator.resetPosition(io.getRotation2d(), MecanumDriveWheelPositions, robotPose2d);
      m_InitializeDFromTag = true;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Navigation", inputs);
    double cl;
    double tl;
    if (m_InitializeDFromTag == false) {
     resetPosition();
    } else {
      MecanumDriveWheelPositions positions = new MecanumDriveWheelPositions(
            driveInputs.frontLeftRad*Constants.Misc.metersPerMotorRad, 
            driveInputs.frontRightRad*Constants.Misc.metersPerMotorRad,
            driveInputs.rearLeftRad*Constants.Misc.metersPerMotorRad,
            driveInputs.rearRightRad*Constants.Misc.metersPerMotorRad);
      m_DrivePoseEstimator.update(io.getRotation2d(), positions);

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
    SmartDashboard.putNumber("updated heading", io.getRotation2d().getDegrees());

    posePublisher.set(pose);

    //double currentHeading = SmartDashboard.getNumber("updated heading", io.getRotation2d().getDegrees());
  }

  public Pose2d getPose2d() {
    return m_DrivePoseEstimator.getEstimatedPosition();
  }

  public static void resetgyro() {
  }
}

// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html.