package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveIO {

    @AutoLog
    public static class DriveIOInputs {
        public double frontLeftRad = 0.0;
        public double frontLeftAppliedVolts = 0.0;
        public double frontLeftCurrentAmps = 0.0;
        public double frontLeftRadPerSec = 0.0;
        public double frontLeftEncoderPosition = 0.0;
        public double frontLeftEncoderVelocity = 0.0;
        
        public double frontRightRad = 0.0;
        public double frontRightAppliedVolts = 0.0;
        public double frontRightCurrentAmps = 0.0;
        public double frontRightRadPerSec = 0.0;
        public double frontRightEncoderPosition = 0.0;
        public double frontRightEncoderVelocity = 0.0;

        public double rearLeftRad = 0.0;
        public double rearLeftAppliedVolts = 0.0;
        public double rearLeftCurrentAmps = 0.0;
        public double rearLeftRadPerSec = 0.0;
        public double rearLeftEncoderPosition = 0.0;
        public double rearLeftEncoderVelocity = 0.0;

        public double rearRightRad = 0.0;
        public double rearRightAppliedVolts = 0.0;
        public double rearRightCurrentAmps = 0.0;
        public double rearRightRadPerSec = 0.0;
        public double rearRightEncoderPosition = 0.0;
        public double rearRightEncoderVelocity = 0.0;

        public double commandedXSpeed = 0.0;
        public double commandedYSpeed = 0.0;
        public double commandedZRotation = 0.0;

        public Rotation2d gyroYaw = new Rotation2d();
    }

    public default void updateInputs(DriveIOInputs io) {};

    public default void driveCartesian(double xSpeed, double ySpeed, double zRotation) {};
}
