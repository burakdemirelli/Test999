/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


// Joystick and Buttons
    public static final int m_Joystick1 = 1;
    public static final int m_Joystick2 = 0;
    public static final int m_YPort = 0;
    public static final int m_XPort = 1;
	public static final int m_ZPort = 2;
	public static final int j_autoAim = 3;
	
    //public int m_turretPort = 0;
	
// Drivetrain
    public static final int frontLeftMotor_1 = 4;
	public static final int rearLeftMotor_2 = 3;
	public static final int frontRightMotor_3 = 2;
	public static final int rearRightMotor_4 = 1;


// SpeedLimiting
    public static final double speedLimiting = 1;

// Intake
	public static final int intake_Supporter = 1;
	public static final double intakeKForward = -1;
	public static final double intakeKReverse = 1;
	public static final double intakeStop = 0;
	public static final double timestamp_Current_0 = 0;

// // Color Detector 
//     //Blue
//         public static final double m_firstColorBlue =0.143;
//         public static final double m_secondColorBlue = 0.427;
//         public static final double m_thirdColorBlue = 0.429;
//     //Green
//         public static final double m_firstColorGreen = 0.197;;
//         public static final double m_secondColorGreen = 0.561;
//         public static final double m_thirdColorGreen = 0.240;
//     //Red
//         public static final double m_firstColorRed = 0.561;
//         public static final double m_secondColorRed = 0.23;
//         public static final double m_thirdColorRed = 0.114;
//     //Yellow
//         public static final double m_firstColorYellow = 0.361;
//         public static final double m_secondColorYellow = 0.524;
//         public static final double m_thirdColorYellow = 0.113;

//Vision
	public static final double height_Target = 2.4955;
	// public static final double height_Target = 2.75;
	public static final double height_Cam = .668;
	// public static final double distance_Target = 5.23;
	public static final double angle_cam = 26.74;
	
//Turret
	public static  int k_turretPort = 0;
	public static  double turretkP = 0.030;
	public static  double turretkI = 0;
	public static  double turretkD = 0;

// Storage
	public static final int k_storagePort = 2;
	public static final int k_feederPort = 3;
	public static final int k_feederBoostPort = 2;

	public static final double storageSpeed = 0.60;
	public static final double feederSpeed = -0.8;
	public static final double feederBoostSpeed = 0.63;
	public static final double storageSpeed2 = -0.9;
	public static final double feederSpeed2 = 0.8;
	public static final double feederBoostSpeed2 = -0.63;
	
    
    // Shooter
    public static final int k_shooterMPort = 0;
    public static final int k_shooterSPort = 3;
    public static final int k_hoodPort = 1;
    public static final double shooterRevSpeed = 0;
	public static final int shooterEncoderA = 4;
	public static final int shooterEncoderB = 3;
	
    public static final double hoodSpeed = 0.6;
	public static final double hoodMoveTime = 0.3;
    
	public static final int PDP_hoodChannel = 0;
	public static final double autoAimP = 0;
	public static final double autoAimI = 0;
	public static final double autoAimD = 0;


	/*
	public static final double shooterkS = 0.103;
	public static final double shooterkV = 0.0125;
	public static final double shooterkA = 0.000256;
	public static final double shooterkD = 0;
	public static final double shooterkI = 0;
	public static double shooterkP = 0.0079;
*/
	public static final double shooterkS = 0.832 ;
	public static final double shooterkV = 0.00059179;
	public static final double shooterkA = 0.000199;
	//167
	// public static final double shooterkP = 6.15e-12;
	public static final double shooterkI = 1.35e-7;
	//public static final double shooterkP =  9.15e-12;
	public static final double shooterkP =  10e-12;
	public static final double shooterkD = 0;
	public static final int drivebaseWidth = 0;
	public static final int drivebaseLength = 0;
	public static final double kPFrontLeftVel = 0;
	public static final double kPRearLeftVel = 0;
	public static final double kPFrontRightVel = 0;
	public static final double kPRearRightVel = 0;
	//63
	//public static double shooterkP = 0.0012;



	// TODO: !!!!!!!!!!!! fill these in
	public static final class Autonomous {
		public static final double kMaxSpeedMetersPerSecond = 0;
		public static final double kMaxAccelerationMetersPerSecondSquared = 0;
		public static final double kMaxAngularSpeedRadiansPerSecond = 0;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = 0;
	
		public static final double kPXController = 0;
		public static final double kPYController = 0;
		public static final double kPThetaController = 0;
	
		//Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
			new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
			kMaxAngularSpeedRadiansPerSecondSquared);
		public static final double kRamseteB = 0;
		public static final double kRamseteZeta = 0;
	
	}


	// TODO: rename to DriveSystem because encoders are also in here
	public static final class DriveMotors {

		public static final boolean kGyroReversed = false;
		public static final double ksVolts = 0;
		public static final double kvVoltSecondsPerMeter = 0;
		public static final double kaVoltSecondsSquaredPerMeter = 0;

		public static final double kTrackwidthMeters = .65;
		public static final DifferentialDriveKinematics kDriveKinematics =
        	new DifferentialDriveKinematics(kTrackwidthMeters);
		public static final double kPDriveVel = 0;




	}

	public static class AutoShooterParameters{
		public static double targetShooterRPM;
		public static double turretAngle;
		public static String targetHoodPosition;
		public static double hoodMoveStartTime;
		
		public static boolean shootReady;
		public static boolean foundTarget;
	}


	public static double initRelativeAngle = 0.0;
}
