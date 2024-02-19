// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.Swerve;
import frc.robot.subsystems.*;

public class SlowMode extends Command {

  private final Swerve swerve;
  private double percentSpeed;

  public SlowMode(Swerve swerve, double percentSpeed) {
    this.swerve = swerve;
    this.percentSpeed = percentSpeed;
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.setSpeed(percentSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}