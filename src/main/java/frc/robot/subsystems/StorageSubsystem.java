/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StorageSubsystem extends SubsystemBase {

  private WPI_VictorSPX storage = new WPI_VictorSPX(Constants.k_storagePort);
 
  public void storageIn() {
    storage.set(Constants.storageSpeed);
    
  }
  public void storageOut(){
    storage.set(Constants.storageSpeed2);
  }

  public void stopEverything() {
    storage.set(0);

  }

  public void runStorage() {
    storage.set(Constants.storageSpeed);
  }

  public void stopStorage() {
    storage.set(0);
  }

  /**
   * Creates a new StorageSubsystem.
   */
  public StorageSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
