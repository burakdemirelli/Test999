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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;


public class AutoAimCommand extends CommandBase {

  private final NetworkTableInstance table = NetworkTableInstance.getDefault();
  private final ShooterSubsystem m_ShooterSubsystem;
  private final TurretSubsystem m_TurretSubsystem;

  private boolean foundTarget = Constants.AutoShooterParameters.foundTarget;
  private boolean shootReady = Constants.AutoShooterParameters.shootReady;

  private final double shooterThreshold =  5;
  private final double  turretThreshold =  0.2;

  private double initRelativeAngle;
  



  public AutoAimCommand(final ShooterSubsystem s_ShooterSubsystem, final TurretSubsystem s_TurretSubsystem){ 
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
    initRelativeAngle = m_TurretSubsystem.getRelativeAngle();
    Constants.initRelativeAngle = initRelativeAngle;


  }

  private double targetShooterSpeed = Constants.AutoShooterParameters.targetShooterRPM;
  private double targetTurretAngle = Constants.AutoShooterParameters.turretAngle;
  //private String targetHoodPosition = ShooterSubsystem.in;
  private String targetHoodPosition = Constants.AutoShooterParameters.targetHoodPosition;
  private double hoodMoveStartTime = Constants.AutoShooterParameters.hoodMoveStartTime;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!foundTarget) {

      // check if target is found
      final double pitch = getPitch();
      final double yaw = getYaw();

      // check if target was found 
      final boolean foundTarget = !(Double.isNaN(pitch) || Double.isNaN(yaw));

      // if still not found, try to move towards target
      if (!foundTarget) 
        m_TurretSubsystem.turnToTarget();


    } else {

      // TODO: calculate all parameters needed to hit target
      final double pitch = getPitch();
      targetShooterSpeed = Double.isNaN(pitch) ? ShooterSubsystem.getRevSpeed() : solveShooterSpeed(pitch);
      targetHoodPosition = ShooterSubsystem.in;
      targetTurretAngle  = m_TurretSubsystem.getRelativeAngle() - getYaw();


      //#region calibrate the shooter constantly
      final boolean hoodReady = Timer.getFPGATimestamp() - hoodMoveStartTime > Constants.hoodMoveTime;
      final boolean turretReady = Math.abs(m_TurretSubsystem.getRelativeAngle() - targetTurretAngle) < turretThreshold;
      


      if (!hoodReady) {
        m_ShooterSubsystem.setHood(targetHoodPosition);
        hoodMoveStartTime = Timer.getFPGATimestamp();
      } else m_ShooterSubsystem.stopHood(targetHoodPosition);
      
      if (!turretReady) {
        System.out.println("TURRET INIT");
        m_TurretSubsystem.turnToAngle(targetTurretAngle);
      }

      shootReady =  hoodReady && turretReady;
      //#endregion

      // TODO: write readyness state to NT
      if(shootReady) SmartDashboard.putBoolean("Ready to launch:", shootReady);
      

    }
  }

   //#region vision

  private final NetworkTable camTable = table.getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
  private final double targetHeight = Constants.height_Target;
  private final double cameraHeight = Constants.height_Cam;
  private final double initialAngle = Constants.angle_cam;
 
  public double getYaw() {
    return camTable.getEntry("targetYaw").getDouble(Double.NaN);
  }

  private double getPitch() {
    return camTable.getEntry("targetPitch").getDouble(Double.NaN);
  }

  public double getDistanceToTarget(final double pitch) {
    final double distance = (targetHeight-cameraHeight) /   Math.tan(Math.toRadians(pitch + initialAngle));
    return distance;
  }

  //#endregion

  

  //#region solving for things

  // TODO: figure out how fast we need to shoot to hit!
  private double solveShooterSpeed(final double pitch) {
    return 1.1 * getDistanceToTarget(pitch);
  }

  //#endregion


  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    // release lock on feeder ??
    Constants.AutoShooterParameters.targetShooterRPM = targetShooterSpeed;
    // reset values to default
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shootReady;
  }
}
