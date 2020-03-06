/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class DriveTrainSubsystem extends SubsystemBase {
  public WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(Constants.frontLeftMotor_1);
  public WPI_TalonSRX rearLeftMotor = new WPI_TalonSRX(Constants.rearLeftMotor_2);
  public WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(Constants.frontRightMotor_3);
  public WPI_TalonSRX rearRightMotor = new WPI_TalonSRX(Constants.rearRightMotor_4);


  private AHRS bodyGyro; 

/*
  private Encoder frontLeftEncoder = new Encoder(DriveMotors.frontLeftEncoderA, DriveMotors.frontLeftEncoderB);
  private Encoder rearLeftEncoder = new Encoder(DriveMotors.rearLeftEncoderA, DriveMotors.rearLeftEncoderB);
  private Encoder frontRightEncoder = new Encoder(DriveMotors.frontRightEncoderA, DriveMotors.frontRightEncoderB);
  private Encoder rearRightEncoder = new Encoder(DriveMotors.rearRightEncoderA, DriveMotors.rearRightEncoderB);
*/

  public DifferentialDrive driveDifferential = new DifferentialDrive(frontLeftMotor, frontRightMotor);

 // public MecanumDrive drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
 // public double MecanumSpeedLimiting = Constants.speedLimiting;


  private DifferentialDriveOdometry m_odometry;


 /*  public void mecanumSet(double y, double x, double z) {
    drive.driveCartesian(y * MecanumSpeedLimiting, x * MecanumSpeedLimiting, z * MecanumSpeedLimiting);
  } */
  
  public void driveDifferential(double x, double z){
    driveDifferential.arcadeDrive(x, z);
  }


  public void stop() {
    driveDifferential.arcadeDrive(0,0);
  }

  
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftDistance(), getRightDistance());
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    zeroHeading();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void arcadeDrive(double fwd, double rot){
    driveDifferential.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    frontLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(-rightVolts);
    driveDifferential.feed();
  }

  public void resetEncoders(){
    frontRightMotor.setSelectedSensorPosition(0);
    frontLeftMotor.setSelectedSensorPosition(0);
    rearLeftMotor.setSelectedSensorPosition(0);
    rearRightMotor.setSelectedSensorPosition(0);
    
  }

  public double getAverageEncoderDistance(){
    return (getLeftDistance() + getRightDistance())/2.0;
  }

  public double getLeftDistance(){

    return ( 
          (-((frontLeftMotor.getSelectedSensorPosition()/4096.0)*(2*Math.PI*(3*2.54)))/100.0) +
          (-((rearLeftMotor.getSelectedSensorPosition()/4096.0)*(2*Math.PI*(3*2.54)))/100.0)
          ) /2.0;
  }

  public double getRightDistance(){
    return ( 
          (((frontRightMotor.getSelectedSensorPosition()/4096.0)*(2*Math.PI*(3*2.54)))/100.0) +
          (((rearRightMotor.getSelectedSensorPosition()/4096.0)*(2*Math.PI*(3*2.54)))/100.0)
          ) /2.0;  
  }

  public void setMaxOutput(double maxOutput){ 
    driveDifferential.setMaxOutput(maxOutput);
  }

  public void zeroHeading(){
    //m_gyro.setYaw(0);
    System.out.println("gyroresetINIT");
    bodyGyro.zeroYaw();
    System.out.println("gyroreset");
  }
  

  public double getHeading() {
    //return Math.IEEEremainder(ypr[0], 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return Math.IEEEremainder(bodyGyro.getAngle(), 360) * (DriveMotors.kGyroReversed ? -1.0 : 1.0);
  }


  /**
   * Creates a new DrivetrainSubsystem.
   */
  public DriveTrainSubsystem(AHRS gyro) {
    // TODO: calibrate and reset encoders
    this.bodyGyro = gyro;
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    rearLeftMotor.follow(frontLeftMotor);
    rearRightMotor.follow(frontRightMotor);
  }

  @Override
  public void periodic() {
    
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(),
                      getRightDistance());
  
              }




}
