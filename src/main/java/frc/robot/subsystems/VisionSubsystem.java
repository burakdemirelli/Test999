/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  public double height_Target = Constants.first_Height + Constants.second_Height;
  public double height_Camera = Constants.height_Cam;
  public double angle_Camera = Constants.angle_cam;
  public double angle_Pitch = Constants.angle_Pitch;
  public double angle_Total = Constants.angle_Total;

  public NetworkTableInstance table = NetworkTableInstance.getDefault();

  public NetworkTable camTable = table.getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-300");

  public NetworkTableEntry yaw = camTable.getEntry("driver_mode");

  public NetworkTableEntry pitch = camTable.getEntry("target_pitch");

  /**
   * Creates a new VisionSubsystem.
   */
  public VisionSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
