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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;


public class AutoAimCommand extends CommandBase {

  private final VisionSubsystem m_VisionSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem; 
  private final PIDController pid = new PIDController(
    Constants.autoAimP,
    Constants.autoAimI,
    Constants.autoAimD
  );
  private final boolean aimed = false;
  private double targetSpeed;
  private final double threshold =  0;

  public AutoAimCommand(VisionSubsystem m_VisionSubsystem, ShooterSubsystem s_ShooterSubsystem){ 
    this.m_VisionSubsystem = m_VisionSubsystem;
    this.m_ShooterSubsystem = s_ShooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_VisionSubsystem);
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // lock feeder

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!aimed) {
      if (Math.abs(m_ShooterSubsystem.getShooterSpeed() - targetSpeed) > threshold ) {
        // change the speed
      } else {
        // unlock feeder
        
      }
    } else {
      // take aim
    }
    
    // turn turret
    // set hood
    // set shooter speed


  }

  private void getDistance() {
    m_VisionSubsystem.angle_Pitch = Math.floor(m_VisionSubsystem.pitch.getDouble(0.0)*100/100d);
    m_VisionSubsystem.angle_Total = Math.toDegrees(m_VisionSubsystem.angle_Pitch + m_VisionSubsystem.angle_Camera);
    SmartDashboard.putNumber("distance", (m_VisionSubsystem.height_Target - m_VisionSubsystem.height_Camera) / Math.tan(m_VisionSubsystem.angle_Total));
    SmartDashboard.putNumber("pitch", m_VisionSubsystem.angle_Pitch);
    SmartDashboard.putNumber("dist", (m_VisionSubsystem.height_Target - m_VisionSubsystem.height_Camera));
    SmartDashboard.putNumber("tan", Math.tan(Math.toDegrees(m_VisionSubsystem.angle_Camera + m_VisionSubsystem.angle_Pitch)));
    



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // release lock on feeder
    // release things???
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
