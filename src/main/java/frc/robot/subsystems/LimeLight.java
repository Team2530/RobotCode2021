// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */

  double tx;
  double ty;
  double ta;
  NetworkTable table;
  int light = 1;

  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    // tx = Double.parseDouble(table.getEntry("tx").getValue().toString());
    // ty = Double.parseDouble(table.getEntry("ty").getValue().toString());
    // ta = Double.parseDouble(table.getEntry("ta").getValue().toString());

    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);
    SmartDashboard.putNumber("light", light);
  }

  public double[] getSphericalPosition(double angle, double height) {
    double[] position = { (Constants.target_Height - height) / (Math.tan(angle + ty)), tx, ty };
    return position;
  }

  public double[] getCartesianPosition(double angle, double height) {
    double[] Sposition = getSphericalPosition(angle, height);
    double[] position = { Sposition[0] * Math.cos(Sposition[1]) * Math.sin(Sposition[2]),
        Sposition[0] * Math.sin(Sposition[1]) * Math.sin(Sposition[2]), Sposition[0] * Math.cos(Sposition[2]) };
    return position;
  }

  public void toggleLights()
  {
    if(light == 1)
    {
      light = 3;
    }
    else
    {
      light = 1;
    
    }
    
    table.getEntry("ledMode").setNumber(light);
  }
}
