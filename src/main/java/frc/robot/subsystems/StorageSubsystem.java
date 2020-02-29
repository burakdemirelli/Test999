/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StorageSubsystem extends SubsystemBase {

  private Victor storage = new Victor(Constants.k_storagePort);
  private Victor feeder = new Victor(Constants.k_feederPort);
  private Victor feederBoost = new Victor(Constants.k_feederBoostPort);

  public void storageIn() {
    storage.set(Constants.storageSpeed);
    
  }
  public void storageOut(){
    storage.set(Constants.storageSpeed2);
  }

  public void stopEverything() {
    storage.set(0);
    feeder.set(0);
    feederBoost.set(0);
  }

  public void runStorage() {
    storage.set(Constants.storageSpeed);
  }

  public void stopStorage() {
    storage.set(0);
  }

  public void runFeederBoost() {
    feederBoost.set(Constants.feederBoostSpeed);
  }

  public void stopFeederBoost() {
    feederBoost.set(0);
  }

  public void runFeeder() {
    feeder.set(Constants.feederSpeed);
  }

  public void stopFeeder() {
    feeder.set(0);
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
