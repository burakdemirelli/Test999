/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HoodToggleCommand;

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

    double initAngle = bodyGyro.getAngle();

    m_TurretSubsystem = new TurretSubsystem(turretGyro, bodyGyro, initAngle);


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
          System.out.println(move);
          if (Math.abs(move) > 0.1) {
            m_TurretSubsystem.set(move);
          } else {
            m_TurretSubsystem.goHome();
          }
        },
       m_TurretSubsystem
      )
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


    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  */
}
