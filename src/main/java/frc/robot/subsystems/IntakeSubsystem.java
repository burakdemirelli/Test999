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

public class IntakeSubsystem extends SubsystemBase {

  public Victor intake_Supporter = new Victor(Constants.intake_Supporter);

  /** 
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {

  }

  public void intakeForward() {
    intake_Supporter.set(Constants.intakeKForward);
  }
  public void intakeReverse() {
    intake_Supporter.set(Constants.intakeKReverse);
  }
  public void intakeStop() {
    intake_Supporter.set(0);
  }

  public void intakeTrigger(double a, double b) {
    if (Math.abs(a) > Math.abs(b)) {
      intake_Supporter.set(a);
    }
    else {
      intake_Supporter.set(b);
    }
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
