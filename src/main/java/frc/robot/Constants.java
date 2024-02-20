// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static class PWM {
    public static final int topLaunch = 0;
    public static final int bottomLaunch = 1;
    public static final int intake = 2;
    public static final int trapScoreTurner = 3;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CANSparkMaxID {

    public static final int FrontLeftDriveMotor = 1;
    public static final int FrontRightDriveMotor = 2;
    public static final int RearLeftDriveMotor = 3;
    public static final int RearRightDriveMotor = 4;
    public static final int topRight = 5;
    public static final int topLeft = 6;
    public static final int intake = 7;
    public static final int staging = 8;
    public static final int launcherLoad = 9;
    
  }

  public static class Misc {
    public static final double intakeSpeed = 0.5;
    public static final double speekerLaunchSpeed = 1;
    public static final double ampLaunchSpeed = 0.5;
    public static final int startAngleDegrees = 0;
    public static final int loadAngleDegrees = 90;
    public static final int dropAngleDegrees = 180;
    public static final double intSpeed = .25;
    
    // Positions of the Drive Wheels relative to the center of the Robot.
    public static final double FrontLeftDriveWheel_Position_X = -0.3;
    public static final double FrontLeftDriveWheel_Position_Y = 0.3;
    public static final double FrontRightDriveWheel_Position_X = 0.3;
    public static final double FrontRightDriveWheel_Position_Y = 0.3;
    public static final double RearLeftDriveWheel_Position_X = -0.3;
    public static final double RearLeftDriveWheel_Position_Y = -0.3;
    public static final double RearRightDriveWheel_Position_X = 0.3;
    public static final double RearRightDriveWheel_Position_Y = -0.3;
  }

  public static class CanID {
    public static final int Pigeon2 = 10;
  }
}
