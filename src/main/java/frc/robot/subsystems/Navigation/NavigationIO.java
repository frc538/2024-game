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

        double gyroYaw;
        double gyroPitch;
        double gyroRoll;

        Rotation2d gyroHeading;

        Pose2d EstimatedPose2d;
    }

    public default void updateInputs(NavigationIOInputs io) {};

    public default void resetGyro(){};
}
