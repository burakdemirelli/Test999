/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private final WPI_VictorSPX turret = new WPI_VictorSPX(Constants.k_turretPort);

  private final double kP = Constants.kP;
  private final double kI = Constants.kI;
  private final double kD = Constants.kD;
  private final PIDController turretPID = new PIDController(kP, kI, kD);

  private final PigeonIMU turretGyro = new PigeonIMU(0);
  public static double [] ypr = new double[3];

  public void Turret() {
    turretPID.setTolerance(2,4);
  }

  public void turretDrive(double joystickValue) {
    turret.set(joystickValue);
  }

  public void turretTurnToAngle(double angle) {
    turretPID.setTolerance(2, 4);

    if(turretPID.atSetpoint() != true) {
      turret.set(
        MathUtil.clamp(
          turretPID.calculate(getGyroAngle(), angle) -0.6, 0.6, angle
        )
      );
    }
  }
  
  public double getGyroAngle() {
    turretGyro.getYawPitchRoll(ypr);
    return ypr[0];
  }

  /**
   * Creates a new Turret.
   */
  public TurretSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
