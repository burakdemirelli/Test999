/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Climb extends SubsystemBase {
  /**
   * Creates a new Cl.
   */
  WPI_VictorSPX esc = new WPI_VictorSPX(ClimbConstants.elevatorPort);
  Victor climbMotor = new Victor(9);



  public Climb() {
    esc.setNeutralMode(NeutralMode.Brake);
   // System.out.println("started braking");
  }


  public void elevatorUp() {
    esc.set(0.3);
  }
  
  public void elevatorDown() {
    esc.set(-0.3);
  }

  public void stop() {
    esc.set(0);
  }
  
  public void climbUp() {
    climbMotor.set(0.3);
  }
  
  public void climbDown() {
    climbMotor.set(-0.3);
  }

  public void climbStop() {
    climbMotor.set(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
