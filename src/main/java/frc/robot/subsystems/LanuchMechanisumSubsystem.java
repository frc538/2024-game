// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class LanuchMechanisumSubsystem extends SubsystemBase {
  CANSparkMax top;
  CANSparkMax top2;
  CANSparkMax bottom;
  CANSparkMax intakeRotate;

  DigitalInput intakeRotateLimitSwitch = new DigitalInput(0);

  
  /** Creates a new LanuchMechanisumSubsystem. */
  public LanuchMechanisumSubsystem() {

    top = new CANSparkMax(Constants.CANIDs.topShoot, MotorType.kBrushless);
    top2 = new CANSparkMax(Constants.CANIDs.top2shoot, MotorType.kBrushless);
    bottom = new CANSparkMax(Constants.CANIDs.intake, MotorType.kBrushless);
    intakeRotate = new CANSparkMax(Constants.CANIDs.intakeRotate, MotorType.kBrushless);
    SmartDashboard.putNumber("Intake Rotate Encoder", intakeRotate.getEncoder().getPosition());


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
    top.set(-1);
    top2.set(-0.85);
    bottom.set(-1);
  }

  public void intake() {
    bottom.set(-0.2);
  }

  public void intakeRotateDown() {
    intakeRotate.set(-0.1);
  }

  public void intakeRotateUp() {
    if (intakeRotateLimitSwitch.get()) {
      intakeRotate.set(0);
    } else {
      intakeRotate.set(0.25);
    }
  }

  public void STOPROTATING() {
    intakeRotate.set(0);
  }

  public void stop() {
    top.set(0);
    top2.set(0);
    bottom.set(0);
  }

  public void spinUp() {
    top.set(-1);
    top2.set(-0.95);
    bottom.set(0);
  }

  @Override
  public void periodic() {
    
  }
}
