// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Revolver extends SubsystemBase {
  
  private static WPI_TalonFX motor_Revolver = new WPI_TalonFX(Constants.motor_revolver_port);

  /** Creates a new Revolver. */
  public Revolver() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRevolverSpeed(double speed) {
    motor_Revolver.set(speed);
  }

}
