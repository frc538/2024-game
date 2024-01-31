// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMechanisum extends SubsystemBase {
  CANSparkMax intake;
  
  /** Creates a new IntakeMechanisum. */
  public IntakeMechanisum() {

    intake = new CANSparkMax(Constants.CANSparkMaxID.intake, MotorType.kBrushless);

    float stallTorque = Constants.misc.stallTorque;
    float freeSpeed = Constants.misc.freeSpeed;

    REVPhysicsSim.getInstance().addSparkMax(intake, stallTorque, freeSpeed);
    
    
  }


  public void intake(){
    intake.set(Constants.misc.intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
