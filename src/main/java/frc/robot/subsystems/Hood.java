// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  //private static WPI_VictorSPX motor_HorizontalHood = new WPI_VictorSPX(Constants.motor_HorizontalHood_Port);
  private static WPI_VictorSPX motor_VerticalHood = new WPI_VictorSPX(Constants.motor_VerticalHood_Port);
  private static WPI_TalonFX motor_Shooter = new WPI_TalonFX(Constants.motor_Shooter_Port);
  /** Creates a new Hood. */
  double flywheelSpeed;
  double hoodPosition = 0; //TODO: Make more reliable method of measurement
  public double hoodTargetAngle = 0;
  public Hood() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheelSpeed = ((motor_Shooter.getSelectedSensorVelocity() * 10 / Constants.ENCODER_TICKS_PER_REVOLUTION) * 2 * Math.PI * Constants.SHOOTER_WHEEL_RADIUS / 100) / Constants.MAX_SHOOTING_VELOCITY;// % of shooting capacity
    hoodPosition += (motor_VerticalHood.getSelectedSensorVelocity() / 5 / Constants.ENCODER_TICKS_PER_REVOLUTION) * Constants.HOOD_GEAR_RATIO * 360;//hood position in degrees
    if (hoodPosition < Constants.MIN_SHOOTING_ANGLE + 1 || hoodPosition > Constants.MAX_SHOOTING_ANGLE - 1 || (hoodPosition > hoodTargetAngle - 1 && hoodPosition < hoodTargetAngle + 1)) {
      motor_VerticalHood.set(0);
      hoodTargetAngle = 0;
    }
  }

  public void hoodRotateSpeed(double vertical) {
    //motor_HorizontalHood.set(horizontal);
    motor_VerticalHood.set(vertical);
  }

  public void autoAimHood (double distance) {
    moveHoodToAngle(Math.atan((Constants.target_Height - Constants.SHOOTER_HEIGHT) / distance)); //Assuming that gravity has a negligible effect
  }

  public void moveHoodToAngle(double angle) {
    hoodTargetAngle = angle;
    if (hoodTargetAngle < hoodPosition) {
      hoodRotateSpeed(-1);
    } else {
      hoodRotateSpeed(1);
    }
  }

  public void flywheelRotateSpeed(double f_speed) {
    motor_Shooter.set(f_speed * (f_speed / flywheelSpeed));
  }

}
