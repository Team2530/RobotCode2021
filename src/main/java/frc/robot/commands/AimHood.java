// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Hood;

public class AimHood extends Command {

  double vertical;

  /** Creates a new TurnHood. */
  Hood m_hood;
  public AimHood(Hood m_hood, double vertical) {
    this.m_hood = m_hood;
    this.vertical = vertical;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hood.hoodRotateSpeed( vertical);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
