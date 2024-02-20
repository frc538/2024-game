// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

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

	if (Robot.isSimulation())
	{
		REVPhysicsSim.getInstance().addSparkMax(top, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(bottom, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(staging, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(pickup, DCMotor.getNEO(1));
	}
  }
public void launchSpeaker(){
  top.set(Constants.Misc.speekerLaunchSpeed);
  pickup.set(Constants.Misc.speekerLaunchSpeed);
}

public void launchAmp(){
  //launch into the amp
  top.set(Constants.Misc.ampLaunchSpeed);
}

public void initaliseLauncher(){
  pickup.set(Constants.Misc.intSpeed);

}


  @Override
  public void periodic(){}
}
