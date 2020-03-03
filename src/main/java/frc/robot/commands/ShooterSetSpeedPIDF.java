/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterSetSpeedPIDF extends PIDCommand {
  /**
   * Creates a new ShooterSetSpeedPIDF.
   */
  private ShooterSubsystem m_shooterSubsystem;
  private static SimpleMotorFeedforward m_SimpleMotorFeedforward= new SimpleMotorFeedforward(Constants.shooterkS, Constants.shooterkV, Constants.shooterkA);
  
  private static double m_motorOutput;

  private boolean isInterruptable;

  public ShooterSetSpeedPIDF(double RPM, ShooterSubsystem shooterSubsystem, boolean isInterruptable){ 
    super(
        // The controller that the command will use
        new PIDController(Constants.shooterkP, Constants.shooterkI, Constants.shooterkD),
        // This should return the measurement
        shooterSubsystem::getRPM,
        // This should return the setpoint (can also be a constant)
        RPM,
        // This uses the output
        output -> {
          // Use the output here
          m_motorOutput = output + m_SimpleMotorFeedforward.calculate(RPM);
          shooterSubsystem.setShooterVoltage(m_motorOutput);
          System.out.println(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(100);
    m_shooterSubsystem = shooterSubsystem;
    this.isInterruptable = isInterruptable;
    //addRequirements(m_shooterSubsystem);

  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();

    
    m_motorOutput = 0;
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(isInterruptable && getController().atSetpoint()) return true;
    else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    if(!isInterruptable) m_shooterSubsystem.stopShooter();
  }
}
