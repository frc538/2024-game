// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.LeftClimberSubsystem;
import frc.robot.commands.Autos;

//import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.IntakeMechanisum;
import frc.robot.subsystems.LanuchMechanisumSubsystem;
import frc.robot.subsystems.LimelightNavigation;
import frc.robot.subsystems.MecanumDriveSubsystem;
import frc.robot.subsystems.RightClimberSubsystem;
import frc.robot.subsystems.TrapScoreSubsystem;
import frc.robot.subsystems.climberSubsystem;

import java.util.Map;

import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//import frc.robot.subsystems.LimelightNavigation;

//import java.util.Map;

//import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  // private final IntakeMechanisum m_Intakemech = new IntakeMechanisum();
  // private final TrapScoreSubsystem m_TrapScoreSubsystem = new
  // TrapScoreSubsystem();

  private Map<String, RelativeEncoder> Encoders = m_Drive.GetEncoders();

  private final LimelightNavigation m_Navigation = new LimelightNavigation(Encoders);

  private final climberSubsystem m_climber = new climberSubsystem();//has to be after limelight subsystem

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

    m_Navigation.setDefaultCommand(Commands.run(()-> { 
      m_Navigation.ledsOff();
    }, m_Navigation));

    driveJoystick.button(12).onTrue(Commands.run(() -> m_Navigation.resetPosition(), m_Navigation));
    driveJoystick.button(5).whileTrue(Commands.run(() -> m_Drive.alignRedAmp(), m_Drive));
    driveJoystick.button(3).whileTrue(Commands.run(() -> m_Drive.alignRedSpeaker(), m_Drive));
    driveJoystick.button(6).whileTrue(Commands.run(() -> m_Drive.alignBlueAmp(), m_Drive));
    driveJoystick.button(4).whileTrue(Commands.run(() -> m_Drive.alignBlueSpeaker(), m_Drive));
    driveJoystick.button(7).whileTrue(Commands.run(() -> m_Drive.alignRedSource(), m_Drive));
    driveJoystick.button(8).whileTrue(Commands.run(() -> m_Drive.alignBlueSource(), m_Drive));
    driveJoystick.button(10).onTrue(Commands.run(()-> m_Navigation.ledsOn(), m_Navigation));

    // contrJoystick.button(1).onFalse(Commands.run(() -> {
    // m_LaunchMech.launchSpeaker();
    // }, m_LaunchMech));

    // contrJoystick.button(2).onFalse(Commands.run(() -> {
    // m_LaunchMech.launchAmp();
    // },m_LaunchMech));

    // contrJoystick.button(3).whileTrue(Commands.run(() -> {
    // m_Intakemech.intake();
    // },m_Intakemech));

    // contrJoystick.button(3).onFalse(Commands.run(() -> {
    // m_TrapScoreSubsystem.startAngle();
    // },m_TrapScoreSubsystem));

    // contrJoystick.button(4).onFalse(Commands.run(() -> {
    // m_TrapScoreSubsystem.loadAngle();
    // },m_TrapScoreSubsystem));

    // contrJoystick.button(5).onFalse(Commands.run(() -> {
    // m_TrapScoreSubsystem.dropAngle();
    // },m_TrapScoreSubsystem));

    mechanismJoystick.axisGreaterThan(1, 0.5).whileTrue(Commands.startEnd(() -> mLeftClimber.lower(),
        () -> mLeftClimber.stop(), mLeftClimber));
    mechanismJoystick.axisLessThan(1, -0.5).whileTrue(Commands.startEnd(() -> mLeftClimber.raise(),
        () -> mLeftClimber.stop(), mLeftClimber));

    mechanismJoystick.axisGreaterThan(5, 0.5).whileTrue(Commands.startEnd(() -> mRightClimber.lower(),
        () -> mRightClimber.stop(), mRightClimber));
    mechanismJoystick.axisLessThan(5, -0.5).whileTrue(Commands.startEnd(() -> mRightClimber.raise(),
        () -> mRightClimber.stop(), mRightClimber));

    mechanismJoystick.button(1).whileTrue(Commands.startEnd(() -> m_climber.lower(), () -> m_climber.stop(), m_climber));
    mechanismJoystick.button(2).whileTrue(Commands.startEnd(() -> m_climber.raise(), () -> m_climber.stop(), m_climber));

    

    mechanismJoystick.axisGreaterThan(3, 0.5)
        .whileTrue(Commands.startEnd(() -> m_LaunchMech.intake(), () -> m_LaunchMech.stop(), m_LaunchMech));
    mechanismJoystick.axisGreaterThan(2, 0.5).whileTrue(Commands.run(() ->
    m_LaunchMech.spinUp(), m_LaunchMech));
    mechanismJoystick.button(5).onTrue(
        Commands.run(() -> m_LaunchMech.spinUp(), m_LaunchMech).withTimeout(2)
            .andThen(Commands.run(() -> m_LaunchMech.shoot(), m_LaunchMech).withTimeout(0.5))
            .andThen(Commands.run(() -> m_LaunchMech.stop(), m_LaunchMech)));

    if (RobotBase.isSimulation())
      REVPhysicsSim.getInstance().run();

  }

  public Command getAuto()
  {
    return Commands.run(() -> m_Drive.drive(Constants.Autos.maxSpeed,0,0,0),m_Drive).withTimeout(Constants.Autos.driveTimeout)
    //.andThen(Commands.run(() -> m_Drive.alignRedSpeaker(), m_Drive)).withTimeout(Constants.Autos.alignTimeout)
    //.andThen(Commands.run(() -> m_LaunchMech.spinUp(), m_LaunchMech)).withTimeout(0.5)
    //.andThen(Commands.run(() -> m_LaunchMech.shoot(), m_LaunchMech)).withTimeout(0.7)
    .andThen(Commands.runOnce(() -> m_LaunchMech.stop(), m_LaunchMech));
  }

}