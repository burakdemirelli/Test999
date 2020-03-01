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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private static final int threshold = 5;

  private final WPI_VictorSPX turret = new WPI_VictorSPX(Constants.k_turretPort);

  private final double kP = Constants.kP;
  private final double kI = Constants.kI;
  private final double kD = Constants.kD;

  private final PIDController turretPID = new PIDController(kP, kI, kD);
  private final PigeonIMU turretGyro;
  private AHRS bodyGyro;

  private final double home = -90;
  
  private final double[] edges = {-135, 45};

  public TurretSubsystem(PigeonIMU gyro, AHRS bodyGyro){
    turretGyro = gyro;
    turretGyro.setYaw(home);
    this.bodyGyro = bodyGyro;
    turretPID.setTolerance(2,4);

  }

  public void set(double speed) {
    double[] dists = distancesToEdges();
    double dist = speed < 0 ? dists[0] : dists[1];
    if ( dist > threshold) setLimitless(speed);
    else setLimitless(0);
  }


  private double[] distancesToEdges() {
    double loc = getGyroAngle();
    double[] ret = new double[2];
    ret[0] = (loc - edges[0]);
    ret[1] = (edges[1] - loc);
    return ret;
  }

  private void setLimitless(double speed) {
    turret.set(speed);
  }

  public void turnToAngle(double angle) {
    // if (turretPID.atSetpoint() != true) {
    //   turret.set(
    //     MathUtil.clamp(
    //       turretPID.calculate(getRelativeAngle(), angle) - 0.6,
    //       0.6, 
    //       angle
    //     )
    //   );
    // }
  }

  public void goHome() {
    // turnToAngle(home);
    this.setLimitless(0);
  }
  

  public double getRelativeAngle() {
    return getGyroAngle() - bodyGyro.getAngle();
  }

  public double getGyroAngle() {
    double[] ret = new double[3];
    turretGyro.getYawPitchRoll(ret);
    return ret[0];
  }

  private double[] distanceToEdges() {
    return new double[2];
  }



  private double clamp() {

    return 0.0;
  }



  @Override
  public void periodic() {
    System.out.print  (distancesToEdges()[0]);
    System.out.print  (", ");
    System.out.println(distancesToEdges()[1]);

  }
}
