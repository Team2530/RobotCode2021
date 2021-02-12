// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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
  double angleCalc;
  double velocityX;
  double velocityY;
  double timeCalc;
  double aimHeightCalc;
  public double hoodTargetAngle = 0;
  public Hood() {
    /* Factory Default all hardware to prevent unexpected behaviour */
		motor_Shooter.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		motor_Shooter.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
    motor_Shooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,Constants.kPIDLoopIdx, Constants.kTimeoutMs);
											

		/* Config the peak and nominal outputs */
		motor_Shooter.configNominalOutputForward(0, Constants.kTimeoutMs);
		motor_Shooter.configNominalOutputReverse(0, Constants.kTimeoutMs);
		motor_Shooter.configPeakOutputForward(1, Constants.kTimeoutMs);
		motor_Shooter.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		motor_Shooter.config_kF(Constants.kPIDLoopIdx, Constants.motor_Shooter.kF, Constants.kTimeoutMs);
		motor_Shooter.config_kP(Constants.kPIDLoopIdx, Constants.motor_Shooter.kP, Constants.kTimeoutMs);
		motor_Shooter.config_kI(Constants.kPIDLoopIdx, Constants.motor_Shooter.kI, Constants.kTimeoutMs);
		motor_Shooter.config_kD(Constants.kPIDLoopIdx, Constants.motor_Shooter.kD, Constants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheelSpeed = motor_Shooter.getMotorOutputPercent();// % of shooting capacity
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

  public void autoAimHood (double distance) { //! everything must stay in meters
    angleCalc = Math.atan((Constants.target_Height - Constants.SHOOTER_HEIGHT) / distance);
    velocityX = flywheelSpeed * Constants.MAX_SHOOTING_VELOCITY * Math.cos(angleCalc);
    velocityY = flywheelSpeed * Constants.MAX_SHOOTING_VELOCITY * Math.sin(angleCalc);
    timeCalc = distance / velocityX;
    aimHeightCalc = (velocityY * timeCalc) + (0.5 * Constants.gravity * Math.pow(timeCalc, 2));
    moveHoodToAngle(Math.atan((2 * Constants.target_Height - aimHeightCalc) / distance));
  }
  
  public void moveHoodToAngle(double angle) {
    hoodTargetAngle = angle;
    if (hoodTargetAngle < hoodPosition) {
      hoodRotateSpeed(-0.1);
    } else {
      hoodRotateSpeed(0.1);
    }
  }

  public void flywheelRotateSpeed(double f_speed) {
    motor_Shooter.set(ControlMode.PercentOutput, f_speed);
  }
  public void flywheelSpeedSetPercentOutput(double speed){
    motor_Shooter.set(ControlMode.PercentOutput, speed);
  }

}
