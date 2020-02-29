/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  public WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(Constants.frontLeftMotor_1);
  public WPI_TalonSRX rearLeftMotor = new WPI_TalonSRX(Constants.rearLeftMotor_2);
  public WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(Constants.frontRightMotor_3);
  public WPI_TalonSRX rearRightMotor = new WPI_TalonSRX(Constants.rearRightMotor_4);

  public MecanumDrive drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

  public double MecanumSpeedLimiting = Constants.speedLimiting;

  public void mecanumSet(double y, double x, double z) {
    drive.driveCartesian(y * MecanumSpeedLimiting , x * MecanumSpeedLimiting, z * MecanumSpeedLimiting);

    /*
     * double ySpeed =
     * RobotContainer.getJoystickAxis(Constants.ySpeedJoystickPorts_1); double
     * xSpeed = RobotContainer.getJoystickAxis(Constants.xSpeedJoystickPorts_2);
     * double zRotation =
     * RobotContainer.getJoystickAxis(Constants.zRotationPorts_3);
     * 
     * RobotContainer.m_DriveTrain.mecanumSet(ySpeed, xSpeed, zRotation);
     */
  }

  /**
   * Creates a new DrivetrainSubsystem.
   */
  public DriveTrainSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
