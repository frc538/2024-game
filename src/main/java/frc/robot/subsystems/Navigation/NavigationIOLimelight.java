package frc.robot.subsystems.Navigation;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.CanID;

public class NavigationIOLimelight implements NavigationIO {

    private final Pigeon2 m_pigeon2 = new Pigeon2(CanID.Pigeon2);

    public NavigationIOLimelight(){

    }

    @Override
    public void updateInputs(NavigationIOInputs inputs) {
        // Update Pigeon
        inputs.gyroPitch = m_pigeon2.getPitch().getValueAsDouble();
        inputs.gyroYaw = m_pigeon2.getYaw().getValueAsDouble();
        inputs.gyroRoll = m_pigeon2.getRoll().getValueAsDouble();
        inputs.gyroHeading = m_pigeon2.getRotation2d();

        // Update Limelight
        inputs.isVisionMeasurement = LimelightHelpers.getTV(Constants.Misc.LimelightName); 
        if (inputs.isVisionMeasurement == true) {
            inputs.robotPose2d = LimelightHelpers.getBotPose2d_wpiBlue(Constants.Misc.LimelightName);
            inputs.captureLatency = LimelightHelpers.getLatency_Capture("limelight");
            inputs.pipelineLatency = LimelightHelpers.getLatency_Pipeline("limelight");
            inputs.samplesCaptured++;
        }
    }
    
    @Override
    public void resetGyro()
    {
        m_pigeon2.reset();
    }

    @Override
    public void ledControls(Boolean flashBangOn) {
        int ControlValue = flashBangOn ? 1 : 3;
        if (flashBangOn) {
            if (ControlValue == 1) {
                ControlValue = 3;
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ControlValue);
            } else {
                ControlValue = 1;
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ControlValue);
            }
        } else {
            ControlValue = 1;
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ControlValue);
        }
    }
}