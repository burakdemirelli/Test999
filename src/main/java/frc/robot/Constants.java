/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
	public static final double height_Target = 249.55;
	public static final double height_Cam = 69.5;
    // angle_cam = Math.toDegrees(Math.atan((- height_Cam + height_Target) / distance_Target)) - pitch;
	public static final double angle_cam = 14.117653850117721;

//Turret
	public static final int k_turretPort = 0;
	public static final int kP = 0;
	public static final int kI = 0;
	public static final int kD = 0;

// Storage
	public static final int k_storagePort = 0;
	public static final int k_feederPort = 3;
	public static final int k_feederBoostPort = 2;

	public static final double storageSpeed = 0.65;
	public static final double feederSpeed = -0.8;
	public static final double feederBoostSpeed = 0.45;
	public static final double storageSpeed2 = -0.9;
	public static final double feederSpeed2 = 0.8;
	public static final double feederBoostSpeed2 = -0.45;
	
    
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


	public static final double shooterkS = -0.172;
	public static final double shooterkV = 0.000207;
	public static final double shooterkA = 3.66e-6;
	public static final double shooterkD = 0;
	public static final double shooterkI = 0;
	public static double shooterkP = 3.19e-127;
/*
	public static final double shooterkS = 0.775;
	public static final double shooterkV = 0.000615;	
	public static final double shooterkA = 6.18e-6;
	//167
	public static final double shooterkD = 0;
	public static final double shooterkI = 1.35e-7;
	public static double shooterkP = 5.0e-5 * 0.83589;
	//63
	//public static double shooterkP = 0.0012;
*/
	

}
