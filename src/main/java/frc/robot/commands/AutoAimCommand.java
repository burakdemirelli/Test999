/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAimCommand extends CommandBase {

  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  /**
   * Creates a new VisionCommand.
 * @param m_VisionSubsystem
   */
  public AutoAimCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_VisionSubsystem);
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_VisionSubsystem.angle_Pitch = Math.floor(m_VisionSubsystem.pitch.getDouble(0.0)*100/100d);
    m_VisionSubsystem.angle_Total = Math.toDegrees(m_VisionSubsystem.angle_Pitch + m_VisionSubsystem.angle_Camera);
    SmartDashboard.putNumber("distance", (m_VisionSubsystem.height_Target - m_VisionSubsystem.height_Camera) / Math.tan(m_VisionSubsystem.angle_Total));
    SmartDashboard.putNumber("pitch", m_VisionSubsystem.angle_Pitch);
    SmartDashboard.putNumber("dist", (m_VisionSubsystem.height_Target - m_VisionSubsystem.height_Camera));
    SmartDashboard.putNumber("tan", Math.tan(Math.toDegrees(m_VisionSubsystem.angle_Camera + m_VisionSubsystem.angle_Pitch)));

    // rev shooter
    // turn turret
    // set hood
    // set shooter speed
    // shoot


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
