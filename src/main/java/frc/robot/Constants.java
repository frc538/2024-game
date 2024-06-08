// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class ExtraAlignValues {
		public static final double HeadingDistanceAdjustSetting = 10.0;
		public static final double PowerTurn = 0.5;
		public static final double PowerForward = 0.5;
	}

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static class AprilTags {
		public static final int redAmp = 5;
		public static final int blueAmp = 6;
		public static final int redSpeaker = 4;
		public static final int blueSpeaker = 7;
	}

	public static class CANIDs {
		public static final int FrontLeftDriveMotor = 28;
		public static final int FrontRightDriveMotor = 27;
		public static final int RearLeftDriveMotor = 26;
		public static final int RearRighDriveMotor = 25;

		public static final int lowerShooter = 34;
		public static final int upperShooter = 33;

		public static final int leftClimber = 14;
		public static final int rightClimber = 13;

		public static final int intake = 32;
		public static final int intakeRotate = 31;
	}

	public static class Misc {
		public static final double intakeSpeed = 0.5;
		public static final double speekerLaunchSpeed = 1;
		public static final double ampLaunchSpeed = 0.5;
		public static final int startAngleDegrees = 0;
		public static final int loadAngleDegrees = 90;
		public static final int dropAngleDegrees = 180;
		public static final float stallTorque = 2.6f;
		public static final float freeSpeed = 5676;
		public static final double intSpeed = .25;

		// Positions of the Drive Wheels relative to the center of the Robot.
		public static final Translation2d FrontLeftDriveWheel_Position_Meters = new Translation2d(-0.3, 0.3);
		public static final Translation2d FrontRightDriveWheel_Position_Meters = new Translation2d(0.3, 0.3);
		public static final Translation2d RearLeftDriveWheel_Position_Meters = new Translation2d(-0.3, -0.3);
		public static final Translation2d RearRightDriveWheel_Position_Meters = new Translation2d(0.3, -0.3);

		// Motor encoder - 42 ticks per rev
		// Gear ratios are one of the following:
		// 12.75:1   THIS ONE
		// Wheel diameter is 8", 8*.0254*pi m/rev
		public static final double metersPerTick = 8 * (Math.PI * 0.0254) / (12.75);

		public static final double driveDeadzone = 0.125;

		public static final double rotationGain = 0.25;

		public static final String LimelightName = "limelight";

		// climbers
		public static final double climberP = 0.1;
		public static final double climberDeadZone = 5;
		public static final double climberJoystickDeadzone = 0.1;
	}

	public static class CanID {
		public static final int Pigeon2 = 1;
	}

	public static class Autos {
		public static final double maxSpeed = 0.7;
		public static final double distance = 1;
		public static final double alignTimeoutSeconds = 5;
		public static final double driveTimeoutSeconds = 1;
	}

	public static class Alignment {
		public static final double speakerDistanceFeet = 5; // feet
		public static final double speakerAngleOffsetDegrees = 0;
		public static final double ampDistanceFeet = 5; // feet
		public static final double ampAngleOffsetDegrees = 0;
		public static final double sourceDistanceFeet = 5; // feet
		public static final double sourceAngleOffsetDegrees = 0;

	}
}
