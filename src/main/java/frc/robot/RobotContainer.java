// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.IntakeMechanisum;
import frc.robot.subsystems.LanuchMechanisumSubsystem;
import frc.robot.subsystems.MecanumDriveSubsystem;
import frc.robot.subsystems.TrapScoreSubsystem;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final MecanumDriveSubsystem m_Drive = new MecanumDriveSubsystem();

  private final LanuchMechanisumSubsystem m_LaunchMech = new LanuchMechanisumSubsystem();
  private final IntakeMechanisum m_Intakemech = new IntakeMechanisum();
  private final TrapScoreSubsystem m_TrapScoreSubsystem = new TrapScoreSubsystem();

      

  private final CommandJoystick contrJoystick = new CommandJoystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    
    
    m_Drive.setDefaultCommand(Commands.run(() -> {
      double forwardSpeed = -contrJoystick.getY();
      double rightSpeed = -contrJoystick.getX();
      double rotatinalSpeed = -contrJoystick.getZ();
      m_Drive.drive(forwardSpeed, rightSpeed, rotatinalSpeed);
    }, m_Drive));

  contrJoystick.button(1).onFalse(Commands.startEnd(() -> m_LaunchMech.launchSpeaker(),
  () -> m_LaunchMech.stop(), m_LaunchMech).withTimeout(5));

    contrJoystick.button(2).onFalse(Commands.run(() -> {
      m_LaunchMech.launchAmp();
    },m_LaunchMech));

    contrJoystick.button(3).whileTrue(Commands.run(() -> {
      m_Intakemech.intake();
    },m_Intakemech));

    contrJoystick.button(3).onFalse(Commands.run(() -> {
      m_TrapScoreSubsystem.startAngle();
    },m_TrapScoreSubsystem));

    contrJoystick.button(4).onFalse(Commands.run(() -> {
      m_TrapScoreSubsystem.loadAngle();
    },m_TrapScoreSubsystem));

    contrJoystick.button(5).onFalse(Commands.run(() -> {
      m_TrapScoreSubsystem.dropAngle();
    },m_TrapScoreSubsystem));

    contrJoystick.button(12).whileTrue(Commands.run(() -> {
     m_LaunchMech.eject(); 
    },m_LaunchMech));


    if (RobotBase.isSimulation()) REVPhysicsSim.getInstance().run();
   
  }

}
