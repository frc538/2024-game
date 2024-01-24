// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

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
  MecanumDriveOdometry m_DriveOdometry;

  /** Creates a new LimelightNavigation. */
  public LimelightNavigation() {
    Map<String, RelativeEncoder> Encoders = new MecanumDriveSubsystem().GetEncoders(); //BUGBUG: Get this in RobotContainer, pass to this subsystem.
    m_FrontLeftWheel_Endocer	= Encoders.get("Front Left");
    m_FrontRightWheel_Encoder	= Encoders.get("Front Right");
    m_RearLeftWheel_Encoder		= Encoders.get("Rear Left");
    m_RearRightWheel_Encoder	= Encoders.get("Rear Right");


    m_FrontLeftWheel_Position	= new Translation2d(Constants.Misc.FrontLeftDriveWheel_Position_X, Constants.Misc.FrontLeftDriveWheel_Position_Y);
    m_FrontRightWheel_Position	= new Translation2d(Constants.Misc.FrontRightDriveWheel_Position_X, Constants.Misc.FrontRightDriveWheel_Position_Y);
    m_RearLeftWheel_Position	= new Translation2d(Constants.Misc.RearLeftDriveWheel_Position_X, Constants.Misc.RearRightDriveWheel_Position_Y);
    m_RearRightWheel_Position	= new Translation2d(Constants.Misc.RearRightDriveWheel_Position_X, Constants.Misc.RearRightDriveWheel_Position_Y);

    m_Kinematics	= new MecanumDriveKinematics(m_FrontLeftWheel_Position, m_FrontRightWheel_Position, m_RearLeftWheel_Position, m_RearRightWheel_Position);
    m_DriveOdometry	= new MecanumDriveOdometry(m_Kinematics, null, new MecanumDriveWheelPositions(
    	m_FrontLeftWheel_Endocer.getPosition(), m_FrontRightWheel_Encoder.getPosition(), m_RearLeftWheel_Encoder.getPosition(), m_RearRightWheel_Encoder.getPosition()
    ));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html