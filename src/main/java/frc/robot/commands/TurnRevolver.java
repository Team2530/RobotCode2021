// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Revolver;

public class TurnRevolver extends CommandBase {
  /** Creates a new TurnRevolver. */
  Revolver m_revolver;
  public TurnRevolver(Revolver m_revolver) {
    this.m_revolver = m_revolver;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_revolver);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_revolver.setRevolverSpeed(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_revolver.setRevolverSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
