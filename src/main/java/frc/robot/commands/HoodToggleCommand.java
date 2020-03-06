/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodToggleCommand extends CommandBase {
  private final ShooterSubsystem m_ShooterSubsystem;

  private double startTime = 0;
  private String initialState;

  public HoodToggleCommand(ShooterSubsystem s_ShooterSubsystem){ 
     m_ShooterSubsystem = s_ShooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      startTime = Timer.getFPGATimestamp();
      initialState = m_ShooterSubsystem.getHoodState();
      if (initialState.equals(ShooterSubsystem.in)) {
        m_ShooterSubsystem.setHood("out");
      } else {
        m_ShooterSubsystem.setHood("in");
      }
      System.out.println("starting hood movement");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - startTime > Constants.hoodMoveTime) {
      end(false);
    }

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.stopHood(initialState.equals(ShooterSubsystem.in)?ShooterSubsystem.out:ShooterSubsystem.in);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !initialState.equals(m_ShooterSubsystem.getHoodState());
  }
}
