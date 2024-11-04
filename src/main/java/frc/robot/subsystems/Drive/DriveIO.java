package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveIO {

    @AutoLog
    public static class DriveIOInputs {
        public double frontLeftRad = 0.0;
        public double frontLeftAppliedVolts = 0.0;
        public double frontLeftCurrentAmps = 0.0;
        public double frontLeftRadPerSec = 0.0;
        
        public double frontRightRad = 0.0;
        public double frontRightAppliedVolts = 0.0;
        public double frontRightCurrentAmps = 0.0;
        public double frontRightRadPerSec = 0.0;

        public double rearLeftRad = 0.0;
        public double rearLeftAppliedVolts = 0.0;
        public double rearLeftCurrentAmps = 0.0;
        public double rearLeftRadPerSec = 0.0;

        public double rearRightRad = 0.0;
        public double rearRightAppliedVolts = 0.0;
        public double rearRightCurrentAmps = 0.0;
        public double rearRightRadPerSec = 0.0;

        public double commandedXSpeed = 0.0;
        public double commandedYSpeed = 0.0;
        public double commandedZRotation = 0.0;
    }

    public default void updateInputs(DriveIOInputs io) {};

    public default void driveCartesian(double xSpeed, double ySpeed, double zRotation) {};
}
