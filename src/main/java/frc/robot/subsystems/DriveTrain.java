/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
// import edu.wpi.first.wpilibj.SPI.Port;
// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.IOException;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;


public class DriveTrain extends SubsystemBase{
  // private static final Port i2c_port_id = null;

  // -------------------- Motors -------------------- \\
  // Left Motors
  WPI_TalonFX motor_left = new WPI_TalonFX(Constants.motor_left_drive_port);
  WPI_TalonFX motor_right = new WPI_TalonFX(Constants.motor_right_drive_port);
  WPI_TalonSRX motor_temp = new WPI_TalonSRX(Constants.motor_revolver_port);
  AHRS ahrs = new AHRS();


  public DifferentialDrive differentialDrive;
  public DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.WHEEL_DISTANCE);
  DifferentialDriveOdometry m_odometry;
  public final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

  public final PIDController m_leftPIDController = new PIDController(Constants.PIDleftDrive.kP,Constants.PIDleftDrive.kI,Constants.PIDleftDrive.kD);
  public final PIDController m_rightPIDController = new PIDController(Constants.PIDrigthDrive.kP,Constants.PIDrigthDrive.kI,Constants.PIDrigthDrive.kD);

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(0.125);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2);
  
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() 
  {
  motor_left.setInverted(true);
  motor_right.setInverted(false);

  motor_left.setNeutralMode(NeutralMode.Brake);
  motor_right.setNeutralMode(NeutralMode.Brake);
  motor_left.setSelectedSensorPosition(0);
  motor_right.setSelectedSensorPosition(0);
  motor_left.feed();
  motor_right.feed();

  
  m_odometry= new DifferentialDriveOdometry(ahrs.getRotation2d());
  differentialDrive = new DifferentialDrive(motor_left, motor_right);
  differentialDrive.setSafetyEnabled(false);
}
  @Override
  public void periodic() {

    updateOdometry();
  }
  
  public void singleJoystickDrive(double x, double z){
    differentialDrive.arcadeDrive(x, z);
  }
  public void singleJoystickDrivePID(double x, double z){
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(m_speedLimiter.calculate(x) * Constants.kMaxSpeed, 0.0, m_rotLimiter.calculate(z* Constants.kMaxAngularSpeed)));
    SmartDashboard.putString("wheelSpeed",wheelSpeeds.toString());
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

    SmartDashboard.putString("LeftEncoderRate", Double.toString(getLeftEncoderRate()));
    SmartDashboard.putString("RightEncoderRate", Double.toString(getRightEncoderRate()));


    final double leftOutput =
        m_leftPIDController.calculate(getLeftEncoderRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(getRightEncoderRate(), speeds.rightMetersPerSecond);
    SmartDashboard.putString("LeftVoltage", Double.toString(leftOutput + leftFeedforward));
    SmartDashboard.putString("RighttVoltage", Double.toString(rightOutput + rightFeedforward));
    motor_left.setVoltage(leftOutput + leftFeedforward);
    motor_right.setVoltage(rightOutput + rightFeedforward);
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderDistance(), getRightEncoderDistance());
  }

  //NEED TO SET ALL OF THESE CORRECTLY
  public double getLeftEncoderDistance(){
    return motor_left.getSelectedSensorPosition()/Constants.ENCODER_TICKS_PER_REVOLUTION;
  }
  public double getRightEncoderDistance(){
    return motor_right.getSelectedSensorPosition()/Constants.ENCODER_TICKS_PER_REVOLUTION;
  }
  public double getLeftEncoderRate(){
    return motor_left.getSelectedSensorVelocity()/Constants.ENCODER_TICKS_PER_REVOLUTION;
  }
  public double getRightEncoderRate(){
    return motor_right.getSelectedSensorVelocity()/Constants.ENCODER_TICKS_PER_REVOLUTION;
  }

  
  
  public Command getAutonomousCommand(String path) {

    String trajectoryJSON = "paths/YourPath.wpilib.json";
    try{
    Trajectory trajectory = new Trajectory();
    } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.maxVelocityMetersPerSecond,
                             Constants.maxAccelerationMetersPerSecondSq)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_kinematics)
            // Apply the voltage constraint
            .addConstraint(Constants.autoVoltageConstraint);


    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}
