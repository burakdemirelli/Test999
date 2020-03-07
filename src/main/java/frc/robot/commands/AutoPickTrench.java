/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPickTrench extends SequentialCommandGroup {
  /**
   * Creates a new AutoPickTrench.
   */
  public AutoPickTrench(
    DriveTrainSubsystem drivetrain,
    IntakeSubsystem intake,
    TurretSubsystem turret,
    ShooterSubsystem shooter,
    LED led,
    FeederSubsystem feeder,
    StorageSubsystem storage
  ) {
    super(
      // fwd
      new RunCommand(() -> {
          drivetrain.driveMecanum(0, -0.32, 0); 
        }, drivetrain).withTimeout(1.8),
      new AutoDropIntake(drivetrain),
      // rev
      new AutoAim(turret, shooter, led) {
        
        @Override public boolean isFinished() {
          return Math.abs(shooter.getRPM() - shooter.getRequiredRPM()) < 50
              && Math.abs(turret.getYaw()) < 1.5;
        }
      },
      // başlangıçtakileri atma
      new ParallelRaceGroup(
        new AutoAim(turret, shooter, led),
        new RunCommand(() -> {
            feeder.feederOut(0.7);
            System.out.println("feeder running");
        }, feeder),
        new RunCommand(() -> {
            storage.storageIn();
        }, storage).withTimeout(5)
      ),
      // trench toplama
      new ParallelRaceGroup(
        new RunCommand(() -> {
          drivetrain.driveMecanum(0, -0.22, 0); 
        }, drivetrain).withTimeout(2.3),
        new RunCommand(() -> {
          intake.intakeIn();
        }, intake),
        new AutoAim(turret, shooter, led)
      ),
      // ekstraları atma
      new ParallelRaceGroup(
        new AutoAim(turret, shooter, led),
        new RunCommand(() -> {
            feeder.feederOut(0.7);
            System.out.println("feeder running");
        }, feeder),
        new RunCommand(() -> {
            storage.storageIn();
        }, storage).withTimeout(5)
      )

    );
  }
}
