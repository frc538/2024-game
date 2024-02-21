// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanID;

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

  private Pigeon2 pigeon2;
  //private Pigeon2Configuration pigeon2Config;

  int x;

  /** Creates a new LimelightNavigation. */
  public LimelightNavigation(Map<String, RelativeEncoder> encoders) {
    //BUGBUG: Get this in RobotContainer, pass to this subsystem.
	Map<String, RelativeEncoder> Encoders = encoders;
    m_FrontLeftWheel_Endocer	= Encoders.get("Front Left");
    m_FrontRightWheel_Encoder	= Encoders.get("Front Right");
    m_RearLeftWheel_Encoder		= Encoders.get("Rear Left");
    m_RearRightWheel_Encoder	= Encoders.get("Rear Right");


    m_FrontLeftWheel_Position	= new Translation2d(Constants.Misc.FrontLeftDriveWheel_Position_X, Constants.Misc.FrontLeftDriveWheel_Position_Y);
    m_FrontRightWheel_Position	= new Translation2d(Constants.Misc.FrontRightDriveWheel_Position_X, Constants.Misc.FrontRightDriveWheel_Position_Y);
    m_RearLeftWheel_Position	= new Translation2d(Constants.Misc.RearLeftDriveWheel_Position_X, Constants.Misc.RearRightDriveWheel_Position_Y);
    m_RearRightWheel_Position	= new Translation2d(Constants.Misc.RearRightDriveWheel_Position_X, Constants.Misc.RearRightDriveWheel_Position_Y);

    m_Kinematics	= new MecanumDriveKinematics(m_FrontLeftWheel_Position, m_FrontRightWheel_Position, m_RearLeftWheel_Position, m_RearRightWheel_Position);
    m_DriveOdometry	= new MecanumDriveOdometry(m_Kinematics, new Rotation2d(0), new MecanumDriveWheelPositions(
    	m_FrontLeftWheel_Endocer.getPosition(), m_FrontRightWheel_Encoder.getPosition(), m_RearLeftWheel_Encoder.getPosition(), m_RearRightWheel_Encoder.getPosition()
    ));

    pigeon2 = new Pigeon2(CanID.Pigeon2);
    //pigeon2Config = new Pigeon2Configuration();


    x=0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	if ( NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) != 0 )
	{
		double[] robotPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[0]);
		var MecanumDriveWheelPositions = new MecanumDriveWheelPositions(
			m_FrontLeftWheel_Endocer.getPosition(), m_FrontRightWheel_Encoder.getPosition(), m_RearLeftWheel_Encoder.getPosition(), m_RearRightWheel_Encoder.getPosition()
		);

		m_DriveOdometry.resetPosition(pigeon2.getRotation2d(), MecanumDriveWheelPositions, new Pose2d(robotPose[0], robotPose[1], pigeon2.getRotation2d()));
	}

	SmartDashboard.putNumber("Robot X", m_DriveOdometry.getPoseMeters().getX());
	SmartDashboard.putNumber("Robot Y", m_DriveOdometry.getPoseMeters().getY());
  SmartDashboard.putNumber("X", x);
  x++;
  }
}

// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html.