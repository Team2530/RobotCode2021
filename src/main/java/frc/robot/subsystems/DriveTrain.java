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

import frc.robot.Constants;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class DriveTrain {
  // private static final Port i2c_port_id = null;

  // -------------------- Motors -------------------- \\
  // Left Motors
  WPI_TalonFX motor_left = new WPI_TalonFX(Constants.motor_left_drive_port);
  WPI_TalonFX motor_right = new WPI_TalonFX(Constants.motor_right_drive_port);
  DifferentialDrive differentialDrive = new DifferentialDrive(motor_left, motor_right);
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    motor_left.setInverted(false);
    motor_right.setInverted(false);
  }

  public void periodic() {
    m_odometry.update(null, motor_left.getDistance(), motor_right.getDistance());
  }

  public void followTrajectory() {
    String trajectoryJSON = "paths/Barrel.wpilib.json"; //The "barrel racing path" in the AutoNav challenge
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      motor_left.getSelectedSensorVelocity() / 5 / Constants.ENCODER_TICKS_PER_REVOLUTION * Constants.WHEEL_DISTANCE / 100,
      motor_right.getSelectedSensorVelocity() / 5 / Constants.ENCODER_TICKS_PER_REVOLUTION * Constants.WHEEL_DISTANCE / 100);
  }

  public void singleJoystickDrive(double x, double z) {
    differentialDrive.arcadeDrive(z, x); // Don't change, this is supposed to be backwards
  }

  public void stop() {
    differentialDrive.stopMotor();
  }


}
