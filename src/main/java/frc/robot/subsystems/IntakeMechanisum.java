// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMechanisum extends SubsystemBase {
  CANSparkMax intake;
  
  /** Creates a new IntakeMechanisum. */
  public IntakeMechanisum() {

    intake = new CANSparkMax(Constants.CANSparkMaxID.intake, MotorType.kBrushless);

    REVPhysicsSim.getInstance().addSparkMax(intake, DCMotor.getNEO(1));
    
  }


  public void intake(){
    intake.set(Constants.Misc.intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
