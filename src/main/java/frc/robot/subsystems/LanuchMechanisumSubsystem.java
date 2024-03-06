// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class LanuchMechanisumSubsystem extends SubsystemBase {
  CANSparkMax top;
  CANSparkMax bottom;

  
  /** Creates a new LanuchMechanisumSubsystem. */
  public LanuchMechanisumSubsystem() {

    top = new CANSparkMax(Constants.CANIDs.topShoot, MotorType.kBrushless);
    bottom = new CANSparkMax(Constants.CANIDs.bottomShoot, MotorType.kBrushless);

    if (RobotBase.isSimulation()) {
      // REVPhysicsSim.getInstance().addSparkMax(top, DCMotor.getNEO(1));
      // REVPhysicsSim.getInstance().addSparkMax(bottom, DCMotor.getNEO(1));
      // REVPhysicsSim.getInstance().addSparkMax(staging, DCMotor.getNEO(1));
      // REVPhysicsSim.getInstance().addSparkMax(pickup, DCMotor.getNEO(1));
    }
  }

  private void launchSpeaker() {
    top.set(Constants.Misc.speekerLaunchSpeed);
  }

  private void launchAmp() {
    // launch into the amp
    top.set(Constants.Misc.ampLaunchSpeed);
  }

  private void initaliseLauncher() {
    top.set(0);

  }

  public void shoot() {
    top.setVoltage(13);
    bottom.setVoltage(13);
  }

  public void intake() {
    top.set(-0.2);
    bottom.set(-0.2);
  }

  public void stop() {
    top.set(0);
    bottom.set(0);
  }

  public void spinUp() {
    top.setVoltage(13);
    bottom.set(0);
  }

  @Override
  public void periodic() {
  }
}
