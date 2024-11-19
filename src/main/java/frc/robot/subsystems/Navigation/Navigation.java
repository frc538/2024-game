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
import edu.wpi.first.wpilibj.Timer;
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
  boolean flashBangOnOff = true;
  int x;

  // Application IO
  Pose2d EstimatedPose2d = new Pose2d();
  double VisionLatency = 0.0;

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
  }

  /* Used by Drive subsystem to do field oriented */
  public Rotation2d getPoseHeading() {
    return EstimatedPose2d.getRotation();
  }

  public double getRoll() {
    return inputs.gyroRoll;
  }

  public void toggleLEDS() {
    flashBangOnOff = !flashBangOnOff;
    io.ledControls(flashBangOnOff);
  }

  public void resetFieldOrient() {
    io.resetGyro();
    System.out.println("resetfieldorient");
  }

  public void resetPosition() {
    m_InitializeDFromTag = false;
    // m_pigeon2.reset();
    // This method will be called once per scheduler run
    if (LimelightHelpers.getTV(Constants.Misc.LimelightName) == true) {
      Pose2d robotPose2d = LimelightHelpers.getBotPose2d_wpiBlue(Constants.Misc.LimelightName);
      var MecanumDriveWheelPositions = new MecanumDriveWheelPositions(
          driveInputs.frontLeftRad * Constants.Misc.metersPerMotorRad,
          driveInputs.frontRightRad * Constants.Misc.metersPerMotorRad,
          driveInputs.rearLeftRad * Constants.Misc.metersPerMotorRad,
          driveInputs.rearRightRad * Constants.Misc.metersPerMotorRad);
      m_DrivePoseEstimator.resetPosition(inputs.gyroHeading, MecanumDriveWheelPositions, robotPose2d);
      m_InitializeDFromTag = true;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Navigation", inputs);

    MecanumDriveWheelPositions positions = new MecanumDriveWheelPositions(
        driveInputs.frontLeftRad * Constants.Misc.metersPerMotorRad,
        driveInputs.frontRightRad * Constants.Misc.metersPerMotorRad,
        driveInputs.rearLeftRad * Constants.Misc.metersPerMotorRad,
        driveInputs.rearRightRad * Constants.Misc.metersPerMotorRad);
    Logger.recordOutput("Navigation/MecanumDriveWheelPositions",positions);
    m_DrivePoseEstimator.update(inputs.gyroHeading, positions);

    if (inputs.isVisionMeasurement == true) {
      VisionLatency = Timer.getFPGATimestamp() - inputs.pipelineLatency / 1000 - inputs.captureLatency / 1000;
      m_DrivePoseEstimator.addVisionMeasurement(inputs.robotPose2d, VisionLatency);
    }
    
    EstimatedPose2d = m_DrivePoseEstimator.getEstimatedPosition();

    Logger.recordOutput("Navigation/VisionLatency",VisionLatency);
    Logger.recordOutput("Navigation/EstimatedPose2d",EstimatedPose2d);
    Logger.recordOutput("Navigation/initializedFromTag", m_InitializeDFromTag);
  }

  public Pose2d getPose2d() {
    return EstimatedPose2d;
  }

  public static void resetgyro() {
  }
}

// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html.