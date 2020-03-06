/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new AutoShootCommandGroup.
   */
  public AutoShootCommandGroup(ShooterSubsystem s_ShooterSubsystem, TurretSubsystem s_TurretSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super( new AutoAimCommand(s_ShooterSubsystem, s_TurretSubsystem),
            new ShooterSetSpeedPIDF(Constants.AutoShooterParameters.targetShooterRPM, s_ShooterSubsystem, false)
            //new InstantCommand(s_ShooterSubsystem.resetAutoAim(), s_ShooterSubsystem)
    );
  }
}
