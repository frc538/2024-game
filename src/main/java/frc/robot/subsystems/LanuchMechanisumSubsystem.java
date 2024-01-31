// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LanuchMechanisumSubsystem extends SubsystemBase {
    CANSparkMax topLaunch;
    CANSparkMax bottomLaunch;

  /** Creates a new LanuchMechanisumSubsystem. */
  public LanuchMechanisumSubsystem() {

    topLaunch = new CANSparkMax(Constants.CANSparkMaxID.topLaunch, MotorType.kBrushless);
    bottomLaunch = new CANSparkMax(Constants.CANSparkMaxID.bottomLaunch, MotorType.kBrushless);
  

    bottomLaunch.follow(topLaunch);

      float stallTorque = Constants.misc.stallTorque;
      float freeSpeed = Constants.misc.freeSpeed;

      REVPhysicsSim.getInstance().addSparkMax(topLaunch, stallTorque, freeSpeed);
      REVPhysicsSim.getInstance().addSparkMax(topLaunch, stallTorque, freeSpeed);

  }
public void launchSpeaker(){
  topLaunch.set(Constants.misc.speekerLaunchSpeed);
}

public void launchAmp(){
  //launch into the amp
  topLaunch.set(Constants.misc.ampLaunchSpeed);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
