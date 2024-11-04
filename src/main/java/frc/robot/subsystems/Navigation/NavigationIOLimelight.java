package frc.robot.subsystems.Navigation;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.CanID;

public class NavigationIOLimelight implements NavigationIO {

    private final Pigeon2 m_pigeon2 = new Pigeon2(CanID.Pigeon2);

    public NavigationIOLimelight(){

    }


    @Override
    public void updateInputs(NavigationIOInputs inputs) {
        inputs.gyroPitch = m_pigeon2.getPitch().getValueAsDouble();
        inputs.gyroYaw = m_pigeon2.getYaw().getValueAsDouble();
        inputs.gyroRoll = m_pigeon2.getRoll().getValueAsDouble();
    }

    @Override
    public Rotation2d getRotation2d()
    {
        return m_pigeon2.getRotation2d();
    };
    
    @Override
    public void resetGyro()
    {
        m_pigeon2.reset();
    }
}