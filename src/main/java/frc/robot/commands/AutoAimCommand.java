/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;


public class AutoAimCommand extends CommandBase {

  private NetworkTableInstance table = NetworkTableInstance.getDefault();
  private final ShooterSubsystem m_ShooterSubsystem;
  private final TurretSubsystem m_TurretSubsystem;

  private boolean foundTarget = false;
  private boolean shootReady = false;

  private final double shooterThreshold =  5;
  private final double  turretThreshold =  0.2;



  public AutoAimCommand(ShooterSubsystem s_ShooterSubsystem, TurretSubsystem s_TurretSubsystem){ 
    this.m_ShooterSubsystem = s_ShooterSubsystem;
    this.m_TurretSubsystem = s_TurretSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubsystem);
    addRequirements(m_TurretSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // signal to driver that autoaim has not aimed fully


  }

  private double targetShooterSpeed = 0;
  private double targetTurretAngle = 0;
  private String targetHoodPosition = ShooterSubsystem.in;
  private double hoodMoveStartTime = -1;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!foundTarget) {

      // check if target is found
      double pitch = getPitch();
      double yaw = getYaw();

      // check if target was found 
      boolean foundTarget = !(Double.isNaN(pitch) || Double.isNaN(yaw));

      // if still not found, try to move towards target
      if (!foundTarget) 
        m_TurretSubsystem.turnToTarget();


    } else {

      // TODO: calculate all parameters needed to hit target
      double pitch = getPitch();
      targetShooterSpeed = Double.isNaN(pitch) ? ShooterSubsystem.getRevSpeed() : solveShooterSpeed(pitch);
      targetHoodPosition = ShooterSubsystem.in;
      targetTurretAngle  = m_TurretSubsystem.getRelativeAngle() - getYaw();


      //#region calibrate the shooter constantly
      boolean shooterReady = Math.abs(m_ShooterSubsystem.getShooterSpeed() - targetShooterSpeed) < shooterThreshold;
      boolean hoodReady = Timer.getFPGATimestamp() - hoodMoveStartTime > Constants.hoodMoveTime;
      boolean turretReady = Math.abs(m_TurretSubsystem.getRelativeAngle() - targetTurretAngle) < turretThreshold;
      
      if (!shooterReady) {
        m_ShooterSubsystem.setShooter(targetShooterSpeed);
      }

      if (!hoodReady) {
        m_ShooterSubsystem.setHood(targetHoodPosition);
        hoodMoveStartTime = Timer.getFPGATimestamp();
      } else m_ShooterSubsystem.stopHood(targetHoodPosition);
      
      if (!turretReady) {
        m_TurretSubsystem.turnToAngle(targetTurretAngle);
      }

      shootReady = shooterReady && hoodReady && turretReady;
      //#endregion

      // TODO: write readyness state to NT
      

    }
  }

   //#region vision

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

  //#endregion


  //#region solving for things

  // TODO: figure out how fast we need to shoot to hit!
  private double solveShooterSpeed(double pitch) {
    return 1.1 * getDistanceToTarget(pitch);
  }

  //#endregion


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // release lock on feeder ??


    // reset values to default
    foundTarget = false;

    targetShooterSpeed = 0;
    targetTurretAngle = 0;
    targetHoodPosition = ShooterSubsystem.in;
    hoodMoveStartTime = -1;
    
    shootReady = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
