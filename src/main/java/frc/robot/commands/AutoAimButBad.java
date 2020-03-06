/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoAimButBad extends ParallelCommandGroup {

  private ShooterSubsystem shooter;
  private TurretSubsystem turret;
  /**
   * Creates a new AutoAimButBad.
   */
  public AutoAimButBad(TurretSubsystem turret, ShooterSubsystem shooter) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      // aim towards thing
      new RunCommand(
        () -> {
          turret.turretAuto();
        },
        turret
      ),
      new ShooterSetSpeedPIDF(shooter, true)
      
      // start shooter motor
    );
    this.turret = turret;
    this.shooter = shooter;
  }

  @Override
  public void end(boolean interrupted) {
    turret.set(0);
    shooter.stopEverything();
  }
}
