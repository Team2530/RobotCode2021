// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class AimHood extends CommandBase {

  double horizontal;
  double vertical;

  /** Creates a new TurnHood. */
  Hood m_hood;
  public AimHood(Hood m_hood, double horizontal, double vertical) {
    this.m_hood = m_hood;
    this.horizontal = horizontal;
    this.vertical = vertical;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hood.hoodRotateSpeed(horizontal, vertical);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hood.hoodRotateSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
