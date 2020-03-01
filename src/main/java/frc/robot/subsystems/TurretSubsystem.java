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

  private final double kP = -Constants.kP;
  private final double kI = Constants.kI;
  private final double kD = Constants.kD;

  private final PIDController turretPID = new PIDController(kP, kI, kD);
  private final PigeonIMU turretGyro;
  private AHRS bodyGyro;
  private double initAngle;

  private final double home = -90;
  
  private final double[] edges = {-100, 45};

  public TurretSubsystem(PigeonIMU gyro, AHRS bodyGyro, double initAngle){
    turretGyro = gyro;
    turretGyro.setYaw(home);

    this.initAngle = initAngle;
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
     if (turretPID.atSetpoint() != true) {
       turret.set(
         MathUtil.clamp(
           turretPID.calculate(getRelativeAngle(), angle) - 0.6,
           0.6, 
           angle
         )
       );
     }
  }

  public void goHome() {
    // turnToAngle(home);
    this.setLimitless(0);
  }
  

  public double getRelativeAngle() {
    return getGyroAngle() - getBodyAngle();
  }

  public double getBodyAngle(){
    return bodyGyro.getAngle() - initAngle;
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

    /*
    m_VisionSubsystem.angle_Pitch = Math.floor(m_VisionSubsystem.pitch.getDouble(0.0)*100/100d);
    m_VisionSubsystem.angle_Total = Math.toDegrees(m_VisionSubsystem.angle_Pitch + m_VisionSubsystem.angle_Camera);
    SmartDashboard.putNumber("distance", (m_VisionSubsystem.height_Target - m_VisionSubsystem.height_Camera) / Math.tan(m_VisionSubsystem.angle_Total));
    SmartDashboard.putNumber("pitch", m_VisionSubsystem.angle_Pitch);
    SmartDashboard.putNumber("dist", (m_VisionSubsystem.height_Target - m_VisionSubsystem.height_Camera));
    SmartDashboard.putNumber("tan", Math.tan(Math.toDegrees(m_VisionSubsystem.angle_Camera + m_VisionSubsystem.angle_Pitch)));
    */


    double height_Target = Constants.first_Height + Constants.second_Height;
    double height_Camera = Constants.height_Cam;
    double angle_Camera = Constants.angle_cam;
    double angle_Pitch = Constants.angle_Pitch;
    double angle_Total = Constants.angle_Total;

    final double targetHeight = 249.55;
    final double camHeight = 69.5;
  
    NetworkTableInstance table = NetworkTableInstance.getDefault();
  
    NetworkTable camTable = table.getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
  
    
    double yaw = camTable.getEntry("targetYaw").getDouble(0.0);
    double pitch = camTable.getEntry("targetPitch").getDouble(0.0);
     final double initial_pitch =14.117653850117721;

    
    //initial_pitch = Math.toDegrees(Math.atan((- camHeight + targetHeight) / 433)) - pitch;

    double distance = (targetHeight-camHeight) /   Math.tan(Math.toRadians(pitch + initial_pitch));
    System.out.println(distance);
    
    //dist:392.5
    //kamera yükseklik:70
    //hedef yükseklik

    SmartDashboard.putNumber("camera angle", initial_pitch);
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("pitch", pitch);

  

  }
}
 