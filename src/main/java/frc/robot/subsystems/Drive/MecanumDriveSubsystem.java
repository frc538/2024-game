// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Navigation.Navigation;

public class MecanumDriveSubsystem extends SubsystemBase {

  private final DriveIO io;
  public final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  double HeadingDistanceSetting = Constants.ExtraAlignValues.HeadingDistanceAdjustSetting;
  double PowerTurn = Constants.ExtraAlignValues.PowerTurn;
  double PowerForward = Constants.ExtraAlignValues.PowerForward;

  Navigation m_Navigation;
  boolean m_sportMode = false;
  public boolean m_fieldOriented = false;

  AprilTagFieldLayout atfl = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  /** Creates a new MechaniumDrive. */
  public MecanumDriveSubsystem(DriveIO io) {
    this.io = io;
  }

  public void setLimeLightNavigation(Navigation lNavigation) {
    m_Navigation = lNavigation;
  }

  public static double deadzone(double value, double dz) {
    if (value > dz) {
      return value - dz;
    }
    if (value < -dz) {
      return value + dz;
    } else {
      return 0;
    }
  }

  private double driveGainCalculation(double sliderValue) {
    double driveGain;
    if (m_sportMode == true) {
      driveGain = 1;
    } else {
      driveGain = 0.45 * sliderValue + .55;
    }
    Logger.recordOutput("Drive/driveGain", driveGain);
    return driveGain;
  }

  public void drive(double forwardSpeed, double rightSpeed, double rotatinalSpeed, double sliderValue) {
    double driveGain = driveGainCalculation(sliderValue);
    double rotationGain = Constants.Misc.rotationGain;

    if (m_fieldOriented == true) {
      double allianceInversion = 1;
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          allianceInversion = -1;
        }
      }
      var fieldOrientedSpeeds = new ChassisSpeeds(
          deadzone(forwardSpeed * allianceInversion, Constants.Misc.driveDeadzone) * driveGain,
          deadzone(rightSpeed * allianceInversion, Constants.Misc.driveDeadzone) * driveGain,
          deadzone(rotatinalSpeed, Constants.Misc.driveDeadzone) * rotationGain);

      var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldOrientedSpeeds,
          m_Navigation.getPoseHeading().times(-1));

      io.driveCartesian(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
          chassisSpeeds.omegaRadiansPerSecond);

    } else {
      io.driveCartesian(
          deadzone(forwardSpeed, Constants.Misc.driveDeadzone) * driveGain,
          deadzone(rightSpeed, Constants.Misc.driveDeadzone) * driveGain,
          deadzone(rotatinalSpeed, Constants.Misc.driveDeadzone) * rotationGain);
    }
  }

  public void toggleFieldOrient() {
    m_fieldOriented = !m_fieldOriented;
    System.out.printf("field orient: %b%n", m_fieldOriented);
  }

  public void sportMode(boolean setting) {
    m_sportMode = setting;
  }

  public void alignAmp() {
    int targetId = 0;
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        targetId = Constants.AprilTags.redAmp;
      }
      if (ally.get() == Alliance.Blue) {
        targetId = Constants.AprilTags.blueAmp;
      }
    } else {
      // Doesn't do anything/exits out of the function
      return;
    }
    Pose3d pose3d = atfl.getTagPose(targetId).get();

    double targetX = pose3d.getX();
    double targetY = pose3d.getY();
    double desiredRange = 1.2;
    SmartDashboard.putNumber("Tx", targetX);
    SmartDashboard.putNumber("Ty", targetY);
    SmartDashboard.putString("Target", "Amp");

    Logger.recordOutput("Target/Tx", targetX);
    Logger.recordOutput("Target/Ty", targetY);
    Logger.recordOutput("Target/type", "Amp");
    alignrange(desiredRange, targetX, targetY);
  }

  public void alignSpeaker() {
    int targetIdSpeaker = 0;
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        targetIdSpeaker = Constants.AprilTags.redSpeaker;
      }
      if (ally.get() == Alliance.Blue) {
        targetIdSpeaker = Constants.AprilTags.blueSpeaker;
      }
    } else {
      // Doesn't do anything/exits out of the function
      return;
    }
    Pose3d pose3d = atfl.getTagPose(targetIdSpeaker).get();

    double targetX = pose3d.getX();
    double targetY = pose3d.getY();
    double desiredRange = 1.5;
    SmartDashboard.putNumber("Tx", targetX);
    SmartDashboard.putNumber("Ty", targetY);
    SmartDashboard.putString("Target", "Speaker");

    Logger.recordOutput("Target/Tx", targetX);
    Logger.recordOutput("Target/Ty", targetY);
    Logger.recordOutput("Target/type", "Speaker");
    alignrange(desiredRange, targetX, targetY);
  }

  private double normalizeAngle(double angle) {
    if (angle > 180) {
      angle -= 360;
    } else if (angle < -180) {
      angle += 360;
    }
    return angle;
  }

  // point at target from desired range
  public void alignrange(double desiredRange, double targetX, double targetY) {
    Pose2d robotPose = m_Navigation.getPose2d();
    double currentHeading = robotPose.getRotation().getDegrees();

    double desiredHeading = Math.toDegrees(Math.atan2(targetY, targetX));
    double headingError = desiredHeading - currentHeading;

    // Normalizes heading error because we had an issue with aligning.

    headingError = normalizeAngle(headingError);

    double distanceX = targetX - robotPose.getX();
    double distanceY = targetY - robotPose.getY();
    double actualRange = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    double rangeError = desiredRange - actualRange;
    double steer = 0.0f;
    double KPAim = -0.1f;
    double KPDistance = -0.4f;
    double minAim = 0; // 0.05f;

    steer = MathUtil.clamp(deadzone(KPAim * headingError, minAim), -PowerTurn, PowerTurn);

    // This is here to control forward movement until pointed in the general
    // direction of the target
    double HeadingThresholdGain = 0.0;
    if (Math.abs(headingError) > HeadingDistanceSetting) {
      HeadingThresholdGain = 0.0;
    } else {
      HeadingThresholdGain = 1.0;
    }
    double distanceAdjust = MathUtil.clamp(rangeError * KPDistance * HeadingThresholdGain, -PowerForward, PowerForward);

    Logger.recordOutput("Drive/desiredHeading", desiredHeading);
    Logger.recordOutput("Drive/actualHeading", currentHeading);
    Logger.recordOutput("Drive/headingError", headingError);

    Logger.recordOutput("Drive/desiredRange", desiredRange);
    Logger.recordOutput("Drive/actualRange", actualRange);
    Logger.recordOutput("Drive/rangeError", rangeError);

    drive(distanceAdjust, 0, steer, 0);
  }

  // go to desired pos and heading
  public void alignTarget(double desiredx, double desiredy, double desiredHeading) {
    double left = 0;
    double forward = 0;
    double rotationSpeed = 0;
    Pose2d pose = m_Navigation.getPose2d();
    double ydisplacement = pose.getY() - desiredy;
    double xdisplacement = pose.getX() - desiredx;
    double headingdisplacement = pose.getRotation().getDegrees() - desiredHeading;
    double kpHeading = 0.05;
    if (Double.isNaN(xdisplacement)) {
      left = 0;
    } else if (Math.abs(xdisplacement) < 0.06) {
      left = 0;
    } else if (xdisplacement < 0) {
      left = 0.1;
    } else {
      left = -0.1;
    }

    if (Double.isNaN(ydisplacement)) {
      forward = 0;
    } else if (Math.abs(ydisplacement) < 0.06) {
      forward = 0;
    } else if (ydisplacement < 0) {
      forward = 0.1;
    } else {
      forward = -0.1;
    }

    if (Double.isNaN(headingdisplacement)) {
      rotationSpeed = 0;
    } else {
      double error = headingdisplacement;
      if (error > 180.0) {
        error = -(360 - error);
      }
      rotationSpeed = kpHeading * error;
    }

    drive(forward, left, rotationSpeed, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    Logger.recordOutput("Drive/FieldOriented", m_fieldOriented);
  }
}
