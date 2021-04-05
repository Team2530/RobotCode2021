// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import java.security.KeyStore.TrustedCertificateEntry;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  // private static WPI_VictorSPX motor_HorizontalHood = new
  // WPI_VictorSPX(Constants.motor_HorizontalHood_Port);
  private static Servo servo = new Servo(Constants.servo_shooter_port);
  public static WPI_TalonFX motor_shooter = new WPI_TalonFX(Constants.motor_shooter_port);
  private static WPI_TalonSRX motor_turret = new WPI_TalonSRX(Constants.motor_turret_port);
  private static DigitalInput halleffect = new DigitalInput(Constants.turret_encoder);
  private static Counter turret_counter = new Counter(halleffect);
  /** Creates a new Hood. */
  double flywheelSpeed;
  double hoodPosition = 0; // TODO: Make more reliable method of measurement
  double angleCalc;
  double velocityX;
  double velocityY;
  double timeCalc;
  double aimHeightCalc;
  double tx;
  double ty;
  double ta;
  NetworkTable table;
  int light = 1;
  public double hoodTargetAngle = 0;
  double servoPos = 0.5;
  boolean targeting = false;

  public Hood() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    motor_shooter.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    motor_shooter.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] */
    motor_shooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    /* Config the peak and nominal outputs */
    motor_shooter.configNominalOutputForward(0, Constants.kTimeoutMs);
    motor_shooter.configNominalOutputReverse(0, Constants.kTimeoutMs);
    motor_shooter.configPeakOutputForward(1, Constants.kTimeoutMs);
    motor_shooter.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    motor_shooter.config_kF(Constants.kPIDLoopIdx, Constants.motor_Shooter.kF, Constants.kTimeoutMs);
    motor_shooter.config_kP(Constants.kPIDLoopIdx, Constants.motor_Shooter.kP, Constants.kTimeoutMs);
    motor_shooter.config_kI(Constants.kPIDLoopIdx, Constants.motor_Shooter.kI, Constants.kTimeoutMs);
    motor_shooter.config_kD(Constants.kPIDLoopIdx, Constants.motor_Shooter.kD, Constants.kTimeoutMs);

    table = NetworkTableInstance.getDefault().getTable("limelight");
    SmartDashboard.putNumber("Servo pos", servoPos);
    light = (int)table.getEntry("ledMode").getDouble(0.0);
  }

  @Override
  public void periodic() {
    // if(motor_turret.get()>0)
    // turret_counter.set
    // else if(motor_turret.get()<0)
    // turret_counter.setReverseDirection(true);
    // SmartDashboard.putBoolean("Raw Sensor Turret", halleffect.get());
    // SmartDashboard.putNumber("Turret encoder", turret_counter.get());
    if (targeting) {
      if (tx > 3) {
        motor_turret.set(ControlMode.PercentOutput, -1);
      } else if (tx < -3) {
        motor_turret.set(ControlMode.PercentOutput, 1);
      } else {
        motor_turret.stopMotor();
      }
    } else {
      motor_turret.stopMotor();
    }
    // This method will be called once per scheduler run

    // if (hoodPosition < Constants.MIN_SHOOTING_ANGLE + 1 || hoodPosition >
    // Constants.MAX_SHOOTING_ANGLE - 1 || (hoodPosition > hoodTargetAngle - 1 &&
    // hoodPosition < hoodTargetAngle + 1)) {
    // setHoodPosition(0);
    // hoodTargetAngle = 0;
    // }
    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);
    
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(light);
    SmartDashboard.putNumber("tx", tx);

    //moveHoodToAngle(ty);
  }
   
  public void toggleAim(){
    targeting = !targeting;
  }
  public void autoAimHood(double distance) { // ! everything must stay in meters
    angleCalc = Math.atan((Constants.target_Height - Constants.SHOOTER_HEIGHT) / distance);
    velocityX = flywheelSpeed * Constants.MAX_SHOOTING_VELOCITY * Math.cos(angleCalc);
    velocityY = flywheelSpeed * Constants.MAX_SHOOTING_VELOCITY * Math.sin(angleCalc);
    timeCalc = distance / velocityX;
    aimHeightCalc = (velocityY * timeCalc) + (0.5 * Constants.gravity * Math.pow(timeCalc, 2));
    moveHoodToAngle(Math.atan((2 * Constants.target_Height - aimHeightCalc) / distance));
  }

  public void moveHoodToAngle(double angle) {
    // 52/0.2
    
    double pos = 0.6 - ((angle) / 300);
    servo.set(pos);
  }

  public void flywheelRotateSpeed(double f_speed) {
    motor_shooter.set(ControlMode.PercentOutput, f_speed);
  }

  public void flywheelSpeedSetPercentOutput(double speed) {
    motor_shooter.set(ControlMode.PercentOutput, speed);
  }

  public void setTurretPower(double speed) {
    motor_turret.set(ControlMode.PercentOutput, speed);
  }
  public void toggleLight(){
    switch(light){
      case 1:
        light = 3;
        break;
      case 3:
        light = 1;
        break;
      default:
        light = 3;
        break;
    }

  }

}