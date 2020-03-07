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

public class FeederSubsystem extends SubsystemBase {

  private Victor feeder = new Victor(Constants.k_feederPort);
  private Victor feederBoost = new Victor(Constants.k_feederBoostPort);

  public void feederIn() {feederIn(1);};
  public void feederIn(double scale) {
    feeder.set(Constants.feederSpeed2*scale);
    feederBoost.set(Constants.feederBoostSpeed2*scale);
  }

  
  public void feederOut() {feederOut(1);};
  public void feederOut(double scale) {
    feeder.set(Constants.feederSpeed*scale);
    feederBoost.set(Constants.feederBoostSpeed*scale);
  }

  public void runFeederBoost() {
    feederBoost.set(Constants.feederBoostSpeed);
  }

  public void stopFeederBoost() {
    feederBoost.set(0);
  }

  public void stopEverything() {
    feeder.set(0);
    feederBoost.set(0);
  }
  public FeederSubsystem() {
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
