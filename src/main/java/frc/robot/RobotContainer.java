/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final static DriveTrainSubsystem m_DriveTrain = new DriveTrainSubsystem();

  private final PigeonIMU turretGyro = new PigeonIMU(0);
  private final AHRS bodyGyro = new AHRS();

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  // private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  private final TurretSubsystem m_TurretSubsystem;

  private final StorageSubsystem m_StorageSubsystem = new StorageSubsystem();

  private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();

  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  private static Joystick operator = new Joystick(Constants.m_Joystick1);

  private static Joystick driver = new Joystick(Constants.m_Joystick2);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    bodyGyro.zeroYaw();
    turretGyro.setYaw(0.0);

    m_TurretSubsystem = new TurretSubsystem(turretGyro, bodyGyro);


    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link Xboxoperator}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  // Drivetrain
  m_DriveTrain.setDefaultCommand(
    new RunCommand ( () -> m_DriveTrain.mecanumSet(
      driver.getRawAxis(Constants.m_YPort), 
      driver.getRawAxis(Constants.m_XPort), 
      driver.getRawAxis(Constants.m_ZPort)), 
      m_DriveTrain));

  //Intake
  m_IntakeSubsystem.setDefaultCommand(
    new RunCommand ( () -> m_IntakeSubsystem.intakeTrigger(
      operator.getRawAxis(2),
       -1*operator.getRawAxis(3)), 
       m_IntakeSubsystem));

  m_TurretSubsystem.setDefaultCommand(
      new RunCommand (
        () -> {
          double move = operator.getRawAxis(4);
          //System.out.println(move);
          if (Math.abs(move) > 0.1) {
            m_TurretSubsystem.set(move);
          } else {
            m_TurretSubsystem.goHome();
          }
        },
       m_TurretSubsystem
      )
    );

    m_ShooterSubsystem.setDefaultCommand(
      new RunCommand(() -> m_ShooterSubsystem.setShooterVoltage(
          operator.getRawAxis(1)*12 * 0.75
      ), m_ShooterSubsystem)
    );
  
  
  
  
    new JoystickButton(operator, 7)
      .whenPressed(new HoodToggleCommand(m_ShooterSubsystem));
  
    new JoystickButton(operator, 5)
    .whileHeld(new InstantCommand(m_StorageSubsystem::storageIn, m_StorageSubsystem))
    .whenReleased(new InstantCommand(m_StorageSubsystem::stopEverything, m_StorageSubsystem));

    new JoystickButton(operator, 6)
    .whileHeld(new InstantCommand(m_StorageSubsystem::storageOut, m_StorageSubsystem))
    .whenReleased(new InstantCommand(m_StorageSubsystem::stopEverything, m_StorageSubsystem));

    new JoystickButton(operator, 1)
    .whileHeld(new InstantCommand(m_FeederSubsystem::feederIn, m_FeederSubsystem))
    .whenReleased(new InstantCommand(m_FeederSubsystem::stopEverything, m_FeederSubsystem));

    new JoystickButton(operator, 2)
      .whileHeld(new InstantCommand(m_FeederSubsystem:: feederOut, m_FeederSubsystem))
      .whenReleased(new InstantCommand(m_FeederSubsystem::stopEverything, m_FeederSubsystem));
    
    new JoystickButton(operator, 3)
      .whileHeld(new IntakeJamRoutine(m_IntakeSubsystem));

    new JoystickButton(driver, 1)
        .toggleWhenPressed(new ShooterSetSpeedPIDF(10650, m_ShooterSubsystem, false));

    new JoystickButton(driver, 2)
        .toggleWhenPressed(new ShooterSetSpeedPIDF(11150, m_ShooterSubsystem, false));
        
    new JoystickButton(driver, 3)
        .toggleWhenPressed(new ShooterSetSpeedPIDF(12000, m_ShooterSubsystem, false));
    
    new JoystickButton(driver, 4)
        .toggleWhenPressed(new ShooterSetSpeedPIDF(11750, m_ShooterSubsystem, false));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
      new Translation2d(-Constants.drivebaseWidth/2, Constants.drivebaseLength/2),  // front  left
      new Translation2d(Constants.drivebaseWidth/2, Constants.drivebaseLength/2),   // front  right
      new Translation2d(-Constants.drivebaseWidth/2, -Constants.drivebaseLength/2), // rear   left
      new Translation2d(Constants.drivebaseWidth/2, -Constants.drivebaseLength/2)); // rear   right

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      DriveMotors.ffS,
      DriveMotors.ffV,
      DriveMotors.ffA
    );
                                                                  

    // Create config for trajectory
    TrajectoryConfig config =
      new TrajectoryConfig(Autonomous.kMaxSpeedMetersPerSecond,
                            Autonomous.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(kinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
          // TODO: generate list of waypoints for each starting position
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config
    );

    MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
        exampleTrajectory,
        bodyGyro::getPose, // TODO: this should be the function that returns the current pose of the robot

        feedforward,
        kinematics,
        
        //Position contollers
        new PIDController(Autonomous.kPXController, 0, 0),
        new PIDController(Autonomous.kPYController, 0, 0),
        new ProfiledPIDController(Autonomous.kPThetaController, 0, 0,
        Autonomous.kThetaControllerConstraints),

        //Needed for normalizing wheel speeds
        Autonomous.kMaxSpeedMetersPerSecond,

        //Velocity PID's
        new PIDController(Constants.kPFrontLeftVel, 0, 0),
        new PIDController(Constants.kPRearLeftVel, 0, 0),
        new PIDController(Constants.kPFrontRightVel, 0, 0),
        new PIDController(Constants.kPRearRightVel, 0, 0),

        m_DriveTrain::getCurrentWheelSpeeds,

        m_DriveTrain::setDriveSpeedControllersVolts, //Consumer for the output motor voltages

        m_DriveTrain
    );

    // Run path following command, then stop at the end.
    return mecanumControllerCommand.andThen(m_DriveTrain::stop);
  }

}
