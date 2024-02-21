// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrapScoreSubsystem extends SubsystemBase {

   Servo trapScoreTurner;

  /** Creates a new TrapScoreSubsystem. */
  public TrapScoreSubsystem() {

    trapScoreTurner = new Servo(Constants.PWM.trapScoreTurner);
  }

  public void startAngle()
  {
    trapScoreTurner.setAngle(Constants.Misc.startAngleDegrees);
  }

  public void loadAngle()
  {
    trapScoreTurner.setAngle(Constants.Misc.loadAngleDegrees);
  }

  public void dropAngle()
  {
    trapScoreTurner.setAngle(Constants.Misc.dropAngleDegrees);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
