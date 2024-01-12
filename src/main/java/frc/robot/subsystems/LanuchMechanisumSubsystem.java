// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LanuchMechanisumSubsystem extends SubsystemBase {
    PWMSparkMax topLaunch;
    PWMSparkMax bottomLaunch;

  /** Creates a new LanuchMechanisumSubsystem. */
  public LanuchMechanisumSubsystem() {

  topLaunch = new PWMSparkMax(Constants.PWM.topLaunch);
  bottomLaunch = new PWMSparkMax(Constants.PWM.bottomLaunch);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
