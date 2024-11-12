package frc.robot.subsystems.Navigation;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface NavigationIO {
    
    @AutoLog
    public static class NavigationIOInputs {
        Translation2d m_FrontLeftWheel_Position = new Translation2d(0.0,0.0);
        Translation2d m_FrontRightWheel_Position = new Translation2d(0.0,0.0);
        Translation2d m_RearLeftWheel_Position = new Translation2d(0.0,0.0);
        Translation2d m_RearRightWheel_Position = new Translation2d(0.0,0.0);

        // Gyro IO
        double gyroYaw = 0.0;
        double gyroPitch = 0.0;
        double gyroRoll = 0.0;
        Rotation2d gyroHeading = new Rotation2d(0.0);

        // Vision IO
        boolean isVisionMeasurement = false;
        Pose2d robotPose2d = new Pose2d();
        double captureLatency = 0.0;
        double pipelineLatency = 0.0;
        int samplesCaptured = 0;

        // Application IO
        Pose2d EstimatedPose2d = new Pose2d();
        double VisionLatency = 0.0;
    }

    public default void updateInputs(NavigationIOInputs io) {};

    public default void resetGyro(){};

    public default void ledControls(Boolean flashBangOn) {};
}
