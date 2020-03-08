/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import javax.annotation.OverridingMethodsMustInvokeSuper;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.*;
import frc.robot.commands.AutonomousDrive.StartingPosition;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousShooter extends SequentialCommandGroup {


  private static double storageInTime = 1;
  private static double storageOutTime = .08;
    /**
     * Creates a new AutonomousDrive.
     */
    public AutonomousShooter(
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
        }, drivetrain).withTimeout(1.2),
      new AutoDropIntake(drivetrain),
      // rev
      new AutoAim(turret, shooter, led) {
        @Override public boolean isFinished() {
          return Math.abs(shooter.getRPM() - shooter.getRequiredRPM()) < 150
              /*&& Math.abs(turret.getYaw()) < 2.5*/;
        }
      },
      new ParallelRaceGroup(
        new AutoAim(turret, shooter, led),
        new RunCommand(() -> {
            feeder.runFeederBoost();
        }, feeder).withTimeout(0.1)
      ),
      new ParallelRaceGroup(
        new AutoAim(turret, shooter, led),
        new RunCommand(() -> {
            feeder.feederOut(0.7);
        }, feeder).withTimeout(1.5*storageInTime)
      ),
      new ParallelRaceGroup(
        new AutoAim(turret, shooter, led),
        new RunCommand(() -> {
            feeder.feederOut(0.7);
        }, feeder),
        new RunCommand(() -> {
            storage.storageIn();
        }, storage).withTimeout(storageInTime)
      ),
      new ParallelRaceGroup(
        new AutoAim(turret, shooter, led),
        new RunCommand(() -> {
            feeder.feederOut(0.7);
        }, feeder),
        new RunCommand(() -> {
            storage.storageOut();
        }, storage).withTimeout(storageOutTime)
      ),
      new ParallelRaceGroup(
        new AutoAim(turret, shooter, led),
        new RunCommand(() -> {
            feeder.feederOut(0.7);
        }, feeder),
        new RunCommand(() -> {
            storage.storageIn();

        }, storage).withTimeout(storageInTime)
      ),
      new ParallelRaceGroup(
        new AutoAim(turret, shooter, led),
        new RunCommand(() -> {
            feeder.feederOut(0.7);
        }, feeder),
        new RunCommand(() -> {
            storage.storageOut();
        }, storage).withTimeout(storageOutTime)
      ),
      new ParallelRaceGroup(
        new AutoAim(turret, shooter, led),
        new RunCommand(() -> {
            feeder.feederOut(0.7);
        }, feeder),
        new RunCommand(() -> {
            storage.storageIn();
        }, storage).withTimeout(storageInTime)
      ),
      new InstantCommand(() -> {
        intake.intakeStop();
        storage.stopEverything();
        feeder.stopEverything();
      }, storage, intake, feeder)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private static Command getAutonomousCommand(DriveTrainSubsystem m_driveTrain, List<Translation2d> listOfMotion ) {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
              DriveMotors.ksVolts,
              DriveMotors.kvVoltSecondsPerMeter,
              DriveMotors.kaVoltSecondsSquaredPerMeter),
              DriveMotors.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
          Autonomous.kMaxSpeedMetersPerSecond,
          Autonomous.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveMotors.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        listOfMotion,
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );


    return null;
  }
}