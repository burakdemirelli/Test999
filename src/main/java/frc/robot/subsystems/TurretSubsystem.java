/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private static final int threshold = 5;

  private final WPI_VictorSPX turret = new WPI_VictorSPX(Constants.k_turretPort);

  private final double kP = Constants.turretkP;
  private final double kI = Constants.turretkI;
  private final double kD = Constants.turretkD;

  private final PIDController turretPID = new PIDController(kP, kI, kD);
  private final PigeonIMU turretGyro;
  private AHRS bodyGyro;
  private double initAngle;

  private final double home = 0;
  
  private final double[] edges = {-90, 120};

  private NetworkTable camTable;


  public TurretSubsystem(PigeonIMU gyro, AHRS bodyGyro, double initAngle, NetworkTable camTable){
    turretGyro = gyro;
    this.camTable = camTable;

    turretGyro.setYaw(home);

    this.bodyGyro = bodyGyro;
    turretPID.setTolerance(2,4);

  }

  public void set(double speed) {
    setLimitless(speed);
  }

  // public void bodyGyrReset(){
  //   bodyGyro.reset();
  // }

  private double[] distancesToEdges() {
    double loc = getRelativeAngle();
    double[] ret = new double[2];
    ret[0] = (loc - edges[0]);
    ret[1] = (edges[1] - loc);
    return ret;
  }

  private void setLimitless(double speed) {
    // System.out.println("speed: " + speed);
    turret.setVoltage(speed*12.0);
  }

  public double getYaw() {
    double yaw =camTable.getEntry("targetYaw").getDouble(Double.NaN); 
    // System.out.println("yaw: " + yaw);
    return yaw;
  }


  public void turretAuto(){
    set(getYaw()*-0.024);
  }

  public void turnToAngle(double angle) {
    angle = clamp(angle);
    if (turretPID.atSetpoint() != true) {
      set(MathUtil.clamp(turretPID.calculate(
        getRelativeAngle(),angle) - 0.6,
        0.6, 
        angle
      ));
    }
  }

  public void turnToTarget() {
    // try to face towards the target-ish
    double angle = 90 - getBodyAngle();
    turnToAngle(angle);
  }

  public void goHome() {
    // turnToAngle(home);
    this.setLimitless(0);
  }
  

  public double getRelativeAngle() {

    return getGyroAngle() - getBodyAngle();
  }

  public double getBodyAngle(){
    return -bodyGyro.getAngle() - initAngle;
  }

  public double getGyroAngle() {
    double[] ret = new double[3];
    turretGyro.getYawPitchRoll(ret);
    return ret[0];
  }


  private double clamp(double angle) {
    angle = angle%360;
    double ret = MathUtil.clamp(angle, edges[0], edges[1]);
    // if (ret != angle) System.out.println("Turret tried to turn to outside bounds");
    return ret;
  }



  @Override
  public void periodic() {
  
    // System.out.println("gyro relative:" + getRelativeAngle());
    // System.out.println("gyro turret:" + getGyroAngle());
    // System.out.println("gyro body:" + bodyGyro.getAngle());
    SmartDashboard.putNumber("relative gyro", getRelativeAngle());
  }
}