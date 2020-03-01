/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {

  public static final String in = "in";
  public static final String out = "out";


  private WPI_TalonSRX shooterMaster = new WPI_TalonSRX(Constants.k_shooterMPort);
  private WPI_VictorSPX shooterSlave = new WPI_VictorSPX(Constants.k_shooterSPort);
  private WPI_VictorSPX hood = new WPI_VictorSPX(Constants.k_hoodPort);
  private Encoder shooterEncoder = new Encoder(new DigitalInput(Constants.shooterEncoderA), new DigitalInput(Constants.shooterEncoderB));

  private boolean hoodRaised = true;

  //#region Shooter

  public void revShooter() {
    // standard speed to set shooter to when preparing for a shot
    shooterMaster.setVoltage(Constants.shooterRevSpeed);

  }

  public void setShooter(double speed) {
    shooterMaster.set(speed);
  }

  public void setShooterVoltage(double voltage) {
    shooterMaster.setVoltage(voltage);
  }

  public void stopShooter() {
      shooterMaster.set(0);
  }

  public double getShooterSpeed() {
    return shooterEncoder.getRate();
  }
  
  public boolean shooterRevved() {
    return true;
  }

  //#endregion

  public void stopEverything() {
      shooterMaster.set(0);
      hood.set(0);
  }

  //#region hood

  public void setHood(String direction) {
    double speed = (direction.equals(in) ?  0.6838 : -1) * Constants.hoodSpeed;
    System.out.println("setting hood to " + speed);
    hood.set(speed);
  }

  //ben biraktim indi
  //                -kaganin annesi

  public void stopHood(String finalState) {
      hood.set(0);
      System.out.println("setting hood to 0");
      hoodRaised = finalState.equals(out);
  }

  public void stopHoodOnly() {
      hood.set(0);
  }

  public String getHoodState() {
      return this.hoodRaised ? out : in;
  }
  //#endregion


  public ShooterSubsystem() {
    super();
    shooterSlave.follow(shooterMaster);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
