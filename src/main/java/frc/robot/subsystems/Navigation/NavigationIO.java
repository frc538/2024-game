package frc.robot.subsystems.Navigation;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface NavigationIO {
    
    @AutoLog
    public static class NavigationIOInputs {
        Translation2d m_FrontLeftWheel_Position;
        Translation2d m_FrontRightWheel_Position;
        Translation2d m_RearLeftWheel_Position;
        Translation2d m_RearRightWheel_Position;

        // Gyro IO
        double gyroYaw;
        double gyroPitch;
        double gyroRoll;
        Rotation2d gyroHeading;

        // Vision IO
        boolean isVisionMeasurement;
        Pose2d robotPose2d;
        double captureLatency;
        double pipelineLatency;
        int samplesCaptured = 0;

        // Application IO
        Pose2d EstimatedPose2d;
        double VisionLatency;
    }

    public default void updateInputs(NavigationIOInputs io) {};

    public default void resetGyro(){};

    public default void ledControls(Boolean flashBangOn) {};
}
