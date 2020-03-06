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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
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
  
  private AHRS bodyGyro = new AHRS();
  private PigeonIMU turretGyro = new PigeonIMU(0);

  public final DriveTrainSubsystem m_DriveTrain;

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  // private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  private final TurretSubsystem m_TurretSubsystem;

  private final StorageSubsystem m_StorageSubsystem = new StorageSubsystem();

  private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();

  private final ShooterSubsystem m_ShooterSubsystem;

  //private final resetTurretHome c_resetTurret;

  private final LED m_LEDSubsystem = new LED();

  private static Joystick operator = new Joystick(Constants.m_Joystick1);

  private static Joystick driver = new Joystick(Constants.m_Joystick2);

  
  public NetworkTableInstance table = NetworkTableInstance.getDefault();
  public NetworkTable camTable = table.getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    bodyGyro.zeroYaw();
    turretGyro.setYaw(0);
    double bodyInitAngle = bodyGyro.getAngle();

    m_ShooterSubsystem = new ShooterSubsystem(camTable);
    m_TurretSubsystem = new TurretSubsystem(turretGyro, bodyGyro, bodyInitAngle, camTable);
    m_DriveTrain = new DriveTrainSubsystem(bodyGyro);

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
    new RunCommand ( () -> m_DriveTrain.driveMecanum(
      driver.getRawAxis(Constants.m_YPort), 
      driver.getRawAxis(Constants.m_XPort), 
      driver.getRawAxis(Constants.m_ZPort)
    ),
    m_DriveTrain)
  );

  //Intake
  m_IntakeSubsystem.setDefaultCommand(
    new RunCommand ( () -> 
      m_IntakeSubsystem.intakeTrigger(
        operator.getRawAxis(2),
        -1*operator.getRawAxis(3)
      ), 
      m_IntakeSubsystem)
    );

  m_TurretSubsystem.setDefaultCommand(
      new RunCommand (
        () -> {
          double move = operator.getRawAxis(4);
          //System.out.println(move);
          if (Math.abs(move) > 0.1) {
            m_TurretSubsystem.set(move);
          } else {
            m_TurretSubsystem.set(0);
          }
        },
       m_TurretSubsystem
      )
    );

    m_ShooterSubsystem.setDefaultCommand(
      new RunCommand(() -> m_ShooterSubsystem.setShooterVoltage(
          operator.getRawAxis(1)*9
      ), m_ShooterSubsystem)
    );


    new JoystickButton(driver, 7)
        .whileHeld(new AutoAimButBad(m_TurretSubsystem, m_ShooterSubsystem, m_LEDSubsystem)
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
        .whenPressed(new InstantCommand(m_LEDSubsystem::turnOffLED, m_LEDSubsystem));

    new JoystickButton(driver, 2)
        .whenPressed(new InstantCommand(m_LEDSubsystem::turnOnLED, m_LEDSubsystem));
    
    new JoystickButton(driver, Constants.j_autoAim)
        .whileHeld(new InstantCommand(m_TurretSubsystem::turretAuto,m_TurretSubsystem));
        //.whenReleased(c_resetTurret);


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /* 
    return new AutonomousDrive(
      m_DriveTrain,
      m_IntakeSubsystem, 
      AutonomousDrive.StartingPosition.RED_FAR_RIGHT
      );*/
      double start = Timer.getFPGATimestamp();
      return new RunCommand(() -> {
        if(Timer.getFPGATimestamp() - start  < 0.55 ) m_DriveTrain.driveMecanum(0, 0.32, 0); 
      }, m_DriveTrain);
  }

}
