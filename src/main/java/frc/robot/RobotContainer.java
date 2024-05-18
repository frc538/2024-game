// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.LeftClimberSubsystem;

import frc.robot.subsystems.IntakeMechanisum;
import frc.robot.subsystems.LanuchMechanisumSubsystem;
import frc.robot.subsystems.LimelightNavigation;
import frc.robot.subsystems.MecanumDriveSubsystem;
import frc.robot.subsystems.RightClimberSubsystem;
import frc.robot.subsystems.climberSubsystem;

import java.security.cert.TrustAnchor;
import java.util.Map;

import javax.print.event.PrintJobEvent;

import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final MecanumDriveSubsystem m_Drive = new MecanumDriveSubsystem();

  private final LanuchMechanisumSubsystem m_LaunchMech = new LanuchMechanisumSubsystem();
  private final LeftClimberSubsystem mLeftClimber = new LeftClimberSubsystem();
  private final RightClimberSubsystem mRightClimber = new RightClimberSubsystem();

  private Map<String, RelativeEncoder> Encoders = m_Drive.GetEncoders();

  private final LimelightNavigation m_Navigation = new LimelightNavigation(Encoders);

  private final climberSubsystem m_climber = new climberSubsystem(mLeftClimber, mRightClimber, m_Navigation);// has to be after
                                                                                               // limelight subsystem

  private final CommandJoystick driveJoystick = new CommandJoystick(0);
  private final CommandJoystick mechanismJoystick = new CommandJoystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_Drive.setLimeLightNavigation(m_Navigation);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_Drive.setDefaultCommand(Commands.run(() -> {
      double forwardSpeed = -driveJoystick.getY();
      double rightSpeed = driveJoystick.getX();
      double rotatinalSpeed = driveJoystick.getZ();
      double slider = driveJoystick.getRawAxis(3);
      m_Drive.drive(forwardSpeed, rightSpeed, rotatinalSpeed, slider);
    }, m_Drive));

    driveJoystick.button(12).onTrue(Commands.runOnce(() -> m_Navigation.resetPosition(), m_Navigation));
    driveJoystick.button(11).onTrue(Commands.runOnce(() -> m_Navigation.resetFieldOrient(), m_Navigation));
    driveJoystick.button(7).whileTrue(Commands.run(() -> m_Drive.alignSpeaker(), m_Drive));
    driveJoystick.button(8).whileTrue(Commands.run(() -> m_Drive.alignAmp(), m_Drive));
    driveJoystick.button(10).onTrue(Commands.run(() -> m_Navigation.ledControls(1), m_Navigation));
    driveJoystick.button(9).onTrue(Commands.runOnce(() -> m_Drive.toggleFieldOrient()));

    driveJoystick.button(5).whileTrue(Commands.run(() -> mLeftClimber.raise(), mLeftClimber));
    driveJoystick.button(3).whileTrue(Commands.run(() -> mLeftClimber.lower(), mLeftClimber));
    driveJoystick.button(3).onFalse(Commands.runOnce(() -> mLeftClimber.stop(), mLeftClimber));
    driveJoystick.button(5).onFalse(Commands.runOnce(() -> mLeftClimber.stop(), mLeftClimber));


    driveJoystick.button(6).whileTrue(Commands.run(() -> mRightClimber.raise(), mLeftClimber));
    driveJoystick.button(4).whileTrue(Commands.run(() -> mRightClimber.lower(), mLeftClimber));
    driveJoystick.button(4).onFalse(Commands.runOnce(() -> mRightClimber.stop(), mLeftClimber));
    driveJoystick.button(6).onFalse(Commands.runOnce(() -> mRightClimber.stop(), mLeftClimber));

    driveJoystick.povDown()
        .whileTrue(Commands.startEnd(() -> m_climber.lower(), () -> m_climber.stop(), m_climber));
    driveJoystick.povUp()
        .whileTrue(Commands.startEnd(() -> m_climber.raise(), () -> m_climber.stop(), m_climber));

    mechanismJoystick.button(1).onTrue(
      Commands.run(() -> m_LaunchMech.intakeRotateDown(), m_LaunchMech));
    mechanismJoystick.button(1).onFalse(Commands.runOnce(() -> m_LaunchMech.STOPROTATING(), m_LaunchMech));

    mechanismJoystick.button(4).whileTrue(
      Commands.run(() -> m_LaunchMech.intakeRotateUp(), m_LaunchMech));
    mechanismJoystick.button(4).onFalse(Commands.runOnce(() -> m_LaunchMech.STOPROTATING(), m_LaunchMech));

    mechanismJoystick.button(6).onTrue(
      Commands.run(() -> m_LaunchMech.spinUp(), m_LaunchMech).withTimeout(1)
      .andThen(Commands.run(() -> m_LaunchMech.shoot())).withTimeout(20)
        .andThen(Commands.runOnce(() -> m_LaunchMech.stop(), m_LaunchMech))
      );
      mechanismJoystick.button(6).onFalse(
      Commands.run(() -> m_LaunchMech.stop(), m_LaunchMech)); 
    

    mechanismJoystick.button(5).onTrue(
      Commands.run(() -> m_LaunchMech.intake(), m_LaunchMech));
    mechanismJoystick.button(5).onFalse(Commands.runOnce(() -> m_LaunchMech.stop(), m_LaunchMech));


    if (RobotBase.isSimulation())
      REVPhysicsSim.getInstance().run();

  }

  public Command autoinit(String selectedAuto) {
    LimelightNavigation.resetgyro();
    m_Navigation.resetFieldOrient();

    if (selectedAuto == "Default Auto") {
      System.out.println("Default auto selected.");
      return Commands.run(() -> m_Drive.drive(Constants.Autos.maxSpeed, 0, 0, 0), m_Drive)
        .withTimeout(Constants.Autos.driveTimeoutSeconds)
        ;
    } else {
      System.out.println("Complex auto selected.");
      System.err.println("Complex auto not created.");

      if (m_Drive.m_fieldOriented == true) {
        System.out.println("Field oriented already on.");
      } else {
        m_Drive.toggleFieldOrient();
      }
      return Commands.run(() -> m_Drive.drive(Constants.Autos.maxSpeed, 0, 0, 0), m_Drive)
          .withTimeout(1)
          // TODO ALIGN YOUR SPEAKER
           .andThen(Commands.run(() -> m_Drive.alignSpeaker(),
           m_Drive)).withTimeout(Constants.Autos.alignTimeoutSeconds)
           .andThen(Commands.run(() -> m_LaunchMech.shoot(),
           m_LaunchMech)).withTimeout(0.7)
          .andThen(Commands.runOnce(() -> m_LaunchMech.stop(), m_LaunchMech));
    }
  }
}