/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodToggleCommand extends CommandBase {

  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private double startTime = 0;
  private String initialState;

  public HoodToggleCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      startTime = Timer.getFPGATimestamp();
      m_ShooterSubsystem.setHood("down");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        if (Timer.getFPGATimestamp() - startTime < Constants.hoodDownTime) {
            m_ShooterSubsystem.stopHood();
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ;
  }
}
