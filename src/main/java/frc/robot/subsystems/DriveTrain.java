/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.SPI.Port;
// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;


public class DriveTrain {
  // private static final Port i2c_port_id = null;

  // -------------------- Motors -------------------- \\
  // Left Motors
  WPI_TalonFX motor_left = new WPI_TalonFX(Constants.motor_left_drive_port);
  WPI_TalonFX motor_right = new WPI_TalonFX(Constants.motor_right_drive_port);
  AHRS ahrs = new AHRS();

  DifferentialDrive differentialDrive = new DifferentialDrive(motor_left, motor_right);
  DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.WHEEL_DISTANCE));//#distance betweeen drive train in cm
  DifferentialDriveOdometry m_odometry;
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() 
  {
  motor_left.setInverted(false);
  motor_right.setInverted(false);

  motor_left.setNeutralMode(NeutralMode.Brake);
  motor_right.setNeutralMode(NeutralMode.Brake);
  
  motor_left.feed();
  motor_right.feed();
  
  m_odometry= new DifferentialDriveOdometry(ahrs.getRotation2d());
  }
  public void singleJoystickDrive(double x, double z){
    differentialDrive.arcadeDrive(x, z);
  }
  public void singleJoystickDrivePID(double x, double z){
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(m_speedLimiter.calculate(x) * Constants.kMaxSpeed, 0.0, m_rotLimiter.calculate(z* Constants.kMaxAngularSpeed)));
    setSpeeds(wheelSpeeds);
  }
  public void stop(){
    differentialDrive.stopMotor();
  }
  public void updateOdometry() {
    m_odometry.update(
        ahrs.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
  }
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(getLeftEncoderRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(getRightEncoderRate(), speeds.rightMetersPerSecond);
    motor_left.setVoltage(leftOutput + leftFeedforward);
    motor_right.setVoltage(rightOutput + rightFeedforward);
  }
  //NEED TO SET ALL OF THESE CORRECTLY
  public double getLeftEncoderDistance(){
    return motor_left.getSelectedSensorPosition();
  }
  public double getRightEncoderDistance(){
    return motor_right.getSelectedSensorPosition();
  }
  public double getLeftEncoderRate(){
    return motor_left.getSelectedSensorVelocity();
  }
  public double getRightEncoderRate(){
    return motor_right.getSelectedSensorVelocity();
  }
  
}
