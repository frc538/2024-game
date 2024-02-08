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
    CANSparkMax topRight;
    CANSparkMax topLeft;
    CANSparkMax staging;
    CANSparkMax launcherLoad;

  /** Creates a new LanuchMechanisumSubsystem. */
  public LanuchMechanisumSubsystem() {

    topRight = new CANSparkMax(Constants.CANSparkMaxID.topRight, MotorType.kBrushless);
    topLeft = new CANSparkMax(Constants.CANSparkMaxID.topLeft, MotorType.kBrushless);
    staging = new CANSparkMax(Constants.CANSparkMaxID.staging, MotorType.kBrushless);
    launcherLoad = new CANSparkMax(Constants.CANSparkMaxID.launcherLoad, MotorType.kBrushless);
  

    topLeft.follow(topRight);

      float stallTorque = Constants.misc.stallTorque;
      float freeSpeed = Constants.misc.freeSpeed;

      REVPhysicsSim.getInstance().addSparkMax(topRight, stallTorque, freeSpeed);
      REVPhysicsSim.getInstance().addSparkMax(topLeft, stallTorque, freeSpeed);
      REVPhysicsSim.getInstance().addSparkMax(staging, stallTorque, freeSpeed);
      REVPhysicsSim.getInstance().addSparkMax(launcherLoad, stallTorque, freeSpeed);
  }
public void launchSpeaker(){
  topRight.set(Constants.misc.speekerLaunchSpeed);
}

public void launchAmp(){
  //launch into the amp
  topRight.set(Constants.misc.ampLaunchSpeed);
}

public void initaliseLauncher(){
  staging.set(Constants.misc.intSpeed);

}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
