// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


  

public class climber extends SubsystemBase {

  CANSparkMax climberMotorA;
  CANSparkMax climberMotorB;

  /** Creates a new climber. */
  public climber() {
  
  
    climberMotorA = new CANSparkMax(Constants.CANIDs.sliderA, MotorType.kBrushless);
    climberMotorB = new CANSparkMax(Constants.CANIDs.sliderB, MotorType.kBrushless);

    climberMotorA.restoreFactoryDefaults();
    climberMotorA.setIdleMode(IdleMode.kBrake);
    climberMotorA.setInverted(false);
    climberMotorA.burnFlash();

    climberMotorB.restoreFactoryDefaults();
    climberMotorB.setIdleMode(IdleMode.kBrake);
    climberMotorB.setInverted(true);
    climberMotorB.burnFlash();

    climberMotorB.follow(climberMotorA);

    if (RobotBase.isSimulation()) {

    REVPhysicsSim.getInstance().addSparkMax(climberMotorA, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(climberMotorB, DCMotor.getNEO(1));
    }
  }

  public void climbUp() {
    climberMotorA.set(Constants.Misc.climbSpeed);
  }

  public void climbDown() {
    climberMotorA.set(-Constants.Misc.climbSpeed);
  }

  public void stop() {
    climberMotorA.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
