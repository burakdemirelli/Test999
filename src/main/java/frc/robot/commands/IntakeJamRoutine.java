/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeJamRoutine extends CommandBase {
  /**
   * Creates a new IntakeJamRoutine.
   */
  private IntakeSubsystem m_intakeSubsystem;
  private double initTime;
  private double currentTime;

  public IntakeJamRoutine(IntakeSubsystem m_intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intakeSubsystem = m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Timer.getFPGATimestamp() - initTime < 0.2) {
      m_intakeSubsystem.intakeReverse();
      System.out.print("intakereverse");
    }
    else if(Timer.getFPGATimestamp() - initTime >= 0.2 && Timer.getFPGATimestamp() - initTime < 0.5) {
      m_intakeSubsystem.intakeForward();
      System.out.print("intakeforward");
    }
    else{
      initTime = Timer.getFPGATimestamp();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.intakeStop();
    System.out.print(interrupted);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
