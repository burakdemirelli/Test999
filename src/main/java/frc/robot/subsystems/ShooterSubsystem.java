/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {

  static final String in = "in";
  static final String out = "out";


  private WPI_TalonSRX shooterMaster = new WPI_TalonSRX(Constants.k_shooterMPort);
  private WPI_TalonSRX shooterSlave = new WPI_TalonSRX(Constants.k_shooterSPort);
  private WPI_VictorSPX hood = new WPI_VictorSPX(Constants.k_hoodPort);
  private Encoder shooterEncoder = new Encoder(Constants.shooterEncoderA, Constants.shooterEncoderB, false);

  private PowerDistributionPanel pdp = new PowerDistributionPanel();

  private boolean hoodRaised = false;

  //#region Shooter

  public void revShooter() {
    // standard speed to set shooter to when preparing for a shot
    shooterMaster.setVoltage(Constants.shooterRevSpeed);
  }

  public void setShooter(double speed) {
    shooterMaster.set(speed);
  }

  public void setShooterVoltage(double speed) {
    shooterMaster.setVoltage(speed);
  }

  public void stopShooter() {
      shooterMaster.set(0);
  }

  public double getShooterSpeed() {
    return shooterEncoder.getRate();
  }

  //#endregion

  public void stopEverything() {
      shooterMaster.set(0);
      hood.set(0);
  }

  //#region hood

  public void setHood(String direction) {
    double speed = (direction.equals(in) ?  1 : -1) * Constants.hoodSpeed;
    hood.set(speed);
    hoodRaised = direction.equals(out);
  }

  public void stopHood() {
      hood.set(0);
  }

  public double getHoodCurrent() {
      return pdp.getCurrent(Constants.PDP_hoodChannel);
  }

  public boolean getHoodIsRaised() {
      return this.hoodRaised;
  }
  //#endregion


  public ShooterSubsystem() {
      shooterSlave.follow(shooterMaster);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
