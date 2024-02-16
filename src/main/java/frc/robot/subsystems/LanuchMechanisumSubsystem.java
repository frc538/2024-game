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
    CANSparkMax top;
    CANSparkMax bottom;
    CANSparkMax staging;
    CANSparkMax pickup;

  /** Creates a new LanuchMechanisumSubsystem. */
  public LanuchMechanisumSubsystem() {

    top = new CANSparkMax(Constants.CANSparkMaxID.topRight, MotorType.kBrushless);
    bottom = new CANSparkMax(Constants.CANSparkMaxID.topLeft, MotorType.kBrushless);
    staging = new CANSparkMax(Constants.CANSparkMaxID.staging, MotorType.kBrushless);
    pickup = new CANSparkMax(Constants.CANSparkMaxID.launcherLoad, MotorType.kBrushless);
  

    bottom.follow(top);
    staging.follow(pickup);

      float stallTorque = Constants.misc.stallTorque;
      float freeSpeed = Constants.misc.freeSpeed;

      REVPhysicsSim.getInstance().addSparkMax(top, stallTorque, freeSpeed);
      REVPhysicsSim.getInstance().addSparkMax(bottom, stallTorque, freeSpeed);
      REVPhysicsSim.getInstance().addSparkMax(staging, stallTorque, freeSpeed);
      REVPhysicsSim.getInstance().addSparkMax(pickup, stallTorque, freeSpeed);
  }
public void launchSpeaker(){
  top.set(Constants.misc.speekerLaunchSpeed);
  pickup.set(Constants.misc.speekerLaunchSpeed);
}

public void launchAmp(){
  //launch into the amp
  top.set(Constants.misc.ampLaunchSpeed);
}

public void initaliseLauncher(){
  pickup.set(Constants.misc.intSpeed);

}


  @Override
  public void periodic(){}
}
