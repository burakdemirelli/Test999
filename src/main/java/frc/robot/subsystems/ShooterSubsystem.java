/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {

  public static final String in = "in";
  public static final String out = "out";


  private WPI_TalonSRX shooterMaster = new WPI_TalonSRX(Constants.k_shooterMPort);
  private WPI_VictorSPX shooterSlave = new WPI_VictorSPX(Constants.k_shooterSPort);
  private WPI_VictorSPX hood = new WPI_VictorSPX(Constants.k_hoodPort);
  
  private Encoder shooterEncoder = new Encoder(new DigitalInput(Constants.shooterEncoderA), new DigitalInput(Constants.shooterEncoderB), true, EncodingType.k1X);


  private boolean hoodRaised = true;


  //GEÇİCİ
  private NetworkTableInstance table = NetworkTableInstance.getDefault();
  private NetworkTable camTable = table.getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
  private final double targetHeight = Constants.height_Target;
  private final double cameraHeight = Constants.height_Cam;
  private final double initialAngle = Constants.angle_cam;

  public double getYaw() {
    return camTable.getEntry("targetYaw").getDouble(Double.NaN);
  }

  private double getPitch() {
    return camTable.getEntry("targetPitch").getDouble(Double.NaN);
  }

  public double getDistanceToTarget(double pitch) {
    double distance = (targetHeight-cameraHeight) /   Math.tan(Math.toRadians(pitch + initialAngle));
    System.out.println(distance);
    return distance;
  }
 //GEÇİCİ SON


  //#region Shooter

  public void revShooter() {
    // standard speed to set shooter to when preparing for a shot
    shooterMaster.setVoltage(Constants.shooterRevSpeed);

  }

  // TODO: add pid system to ensure staying at this speed
  public void setShooter(double speed) {
    shooterMaster.set(speed);
  }

  public double getRPM(){
    return shooterEncoder.getRate()*60/1024.0;
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
    shooterMaster.setInverted(true);
    shooterSlave.setInverted(true);
    shooterSlave.follow(shooterMaster);
    shooterEncoder.setSamplesToAverage(50);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter RPM", Math.floor(getRPM()));
    SmartDashboard.putNumber("distance", getDistanceToTarget(getPitch()));
  }

  public static double getRevSpeed() {
    return Constants.shooterRevSpeed;
  }
}
