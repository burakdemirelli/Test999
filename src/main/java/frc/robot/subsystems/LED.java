/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /**
   * Creates a new LED.
   */
  private Solenoid visionLED = new Solenoid(4);
  public LED() {

  }


  public void turnOnLED(){
    visionLED.set(true);
  }

  public void turnOffLED(){
    visionLED.set(false);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
