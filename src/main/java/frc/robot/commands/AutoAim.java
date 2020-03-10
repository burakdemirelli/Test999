/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class AutoAim extends ParallelCommandGroup {

  private ShooterSubsystem shooter;
  private TurretSubsystem turret;
  private LED led;

  /**
   * Creates a new AutoAimButBad.
   */
  public AutoAim(TurretSubsystem turret, ShooterSubsystem shooter, LED led) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      // aim towards thing
      new RunCommand(
        () -> {
          turret.turretAuto();
          //System.out.println("aiming with turret");
        },
        turret
      ),

      new ShooterSetSpeedPIDF(shooter, true),

      new RunCommand(() -> {
        led.turnOnLED();
      }) {
        @Override
        public void end(boolean interrupted) {
          led.turnOffLED();
        }
      }
      
      // start shooter motor
    );
    this.turret = turret;
    this.shooter = shooter;
    this.led = led;
  }

  @Override
  public void end(boolean interrupted) {
    turret.set(0);
    shooter.stopEverything();
    led.turnOffLED();
  }
}
