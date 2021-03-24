// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import java.security.KeyStore.TrustedCertificateEntry;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  //private static WPI_VictorSPX motor_HorizontalHood = new WPI_VictorSPX(Constants.motor_HorizontalHood_Port);
  private static Servo servo = new Servo(Constants.servo_shooter_port);
  private static WPI_TalonFX motor_Shooter = new WPI_TalonFX(Constants.motor_shooter_port);
  private static WPI_TalonSRX motor_turret = new WPI_TalonSRX(Constants.motor_turret_port);
  private static Counter turret_counter = new Counter(new DigitalInput(Constants.turret_encoder));
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
    if(motor_turret.get()>0)
      turret_counter.setReverseDirection(false);
    else if(motor_turret.get()<0)
      turret_counter.setReverseDirection(true);
    SmartDashboard.putNumber("Raw Turret encoder", turret_counter.get());

    // This method will be called once per scheduler run
    
    if (hoodPosition < Constants.MIN_SHOOTING_ANGLE + 1 || hoodPosition > Constants.MAX_SHOOTING_ANGLE - 1 || (hoodPosition > hoodTargetAngle - 1 && hoodPosition < hoodTargetAngle + 1)) {
      setHoodPosition(0);
      hoodTargetAngle = 0;
    }
  }

  private void setHoodPosition(double pos) {
    //motor_HorizontalHood.set(horizontal);
    servo.set(pos);
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
    //52/0.2
    double pos = 1-(0.2/52*angle);
    servo.set(pos);
  }

  public void flywheelRotateSpeed(double f_speed) {
    motor_Shooter.set(ControlMode.PercentOutput, f_speed);
  }
  public void flywheelSpeedSetPercentOutput(double speed){
    motor_Shooter.set(ControlMode.PercentOutput, speed);
  }


}