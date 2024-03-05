// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MecanumDriveSubsystem;

public final class Autos extends Command {
  private final MecanumDriveSubsystem mecanumDrive;

  public Autos(MecanumDriveSubsystem MDS) {
    this.mecanumDrive = MDS;

    addRequirements(mecanumDrive);
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    double distance = Math.abs(mecanumDrive.GetEncoders().get("Front Left").getPosition());

    return distance > Constants.Autos.distance;
  }

  @Override
  public void initialize() {
    mecanumDrive.drive(Constants.Autos.maxSpeed, 0, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    mecanumDrive.drive(0, 0, 0, 0);
  }
}
