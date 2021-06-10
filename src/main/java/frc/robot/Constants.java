/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.libraries.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
     //! NEED TO BE ACTUALLY SET
    //--------------------Motor Ports--------------------\\
    //DriveTrain Motors
    //ports set up for test drivetrain currently
    public static final int motor_left_drive_port = 2;
    public static final int motor_right_drive_port = 1;
    public static final int motor_revolver_port = 5;
    public static final int motor_flywheel_port = 3;
    public static final int motor_turret_port = 4;
    public static final int motor_hood_port = 6;
    public static final int turret_encoder = 9;
    public enum DriveMotors
    { 
        FL, FR, BL, BR; 
    } 
    //----------Sensor Constants-----------\\
    public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;
    public static final int gyroDrift = 5;
    public static final double sensor_Limelight_Height = 25;//? mounting height in inches
    public static final double HOOD_GEAR_RATIO = 11/72; //TODO: I based this off of the gear tooth ratios, but don't know if that's right
    public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    
    //----------Driving Constants----------\\
    public static final double DRIVE_GEAR_RATIO = 18.57; //?This ratio is the ratio between the encoder and the driven wheels
    public static final double WHEEL_RADIUS = 6*2.54; //!Not diameter radius
    public static final double DISTANCE_PER_PULSE = Constants.ENCODER_TICKS_PER_REVOLUTION * Math.PI * Math.pow(Constants.WHEEL_RADIUS, 2);
    public static final double kS = 0.761;
    public static final double kV = 0.0631;
    public static final double kA = 0.0095;
    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;
    public static final double kMaxSpeed = 2.7;
    public static final double kMaxAngularSpeed = 16*Math.PI;
    public static final double autoDriveMinRampTurnSpeed = 0.6; //The amount of the max speed to start ramping with autonomous turning, from 0 to 1
    public static final double autoDriveRampTurnOffset = -0.85; //The amount to shift the peak of the ramping with autonomous turning. -1 moves the peak to the start, 0 leaves it in the middle, and 1 moves the peak to the end
    public static final double autoDriveMaxTurnSpeed = 0.6; //The maximum speed of autonomous turning, from 0 to 1
    public static final double autoDriveTurnTolerance = 5; //The acceptable distance from the target angle for autonomous turning, in degrees
    public static final Gains PIDleftDrive = new Gains(0.439, 0, 0, 0, 0, 0);
    public static final Gains PIDrigthDrive = new Gains(0.439, 0, 0, 0, 0, 0);
    public static final Gains motor_Shooter= new Gains(1, 0, 0, 0, 0, 0);

    //public static final double ALIGN = 0.025;
    public static final double WHEEL_DISTANCE = Units.inchesToMeters(21);
    

    public static final double tol = 5;
    public static final int setPoint = 1;

    public static final double maxVelocityMetersPerSecond = 2.7; //!This needs to be set
    public static final double maxAccelerationMetersPerSecondSq = 6.47; //!This needs to be set
    public static final double autoVoltageConstraint = 9.5;

    //----------Field Constants----------\\
    public static final double target_Height = 105*2.54/100; //temp test value (in meters)
    public static final double ball_Weight = 0.3125;//? pounds?

    
    //----------Control (Joystick) Constants----------\\
    public static final double deadzone = 0.1;

    //----------Control (Shooting) Constants----------\\
    public static final double SHOOTER_HEIGHT = 20*2.54/100;//from floor to center of opening, in meters. TODO: Get better measurement
    public static final float I = 1;//?moment of inertia
    public static final double SHOOTER_WHEEL_RADIUS = 6*2.54;//cm
    public static final double eff = 0.8;//?effective efficiency percentage
    public static final double MAX_SHOOTING_DISTANCE = 2.54;//meters
    public static final int MAX_SHOOTING_VELOCITY = 20;//meters per sec.
    public static final int MIN_SHOOTING_ANGLE = 0;//degrees
    public static final int MAX_SHOOTING_ANGLE = 30;//degrees
    public static final int IDEAL_SHOOTING_DISTANCE = 190;//cm
    public static final double distanceTolerance = 10; //cm
    public static final double angleTolerance = 10; //degrees

    //----------Control (Elevator) Constants----------\\
    //measurements based around shooter being level  
    //!these are all wrong, hardware didnt know anything
    public static final double bottomLeg = 29.68168*2.54; //cm
    public static final double maxAngle = 45; //degrees (45, 45, 90 triangle)
    public static final double maxHeight = 18.54717*2.54; //cm
    public static final double minAngle = -32; //degrees
    public static final double minHeight = -29.68168*2.54; //cm
	public static final double gravity = -9.81;//meters per second squared
	public static final int DROP_IN_DISTANCE_PER_REVOLUTION = 0;
	

}
