// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private static WPI_VictorSPX motor_HorizontalHood = new WPI_VictorSPX(Constants.motor_HorizontalHood_Port);
  private static WPI_VictorSPX motor_VerticalHood = new WPI_VictorSPX(Constants.motor_VerticalHood_Port);

  /** Creates a new Hood. */
  public Hood() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void hoodRotateSpeed(double horizontal, double vertical) {
    motor_HorizontalHood.set(horizontal);
    motor_VerticalHood.set(vertical);
  }

}
