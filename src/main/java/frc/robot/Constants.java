// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kVictorDeadband = 0.01f;

  public static final class DrivetrainConstants {
    public static final int m_LeftDrivePrimaryPort = 6;
    public static final int m_LeftDriveFollowerPort = 7;
    public static final int m_RightDrivePrimaryPort = 8;
    public static final int m_RightDriveFollowerPort = 9;

    public static final double linear_SlewRateLimit = 0.2;
    public static final double turn_SlewRateLimit = 0.2;

    // Drive with joystick speeds
    public static final double default_TurnCoefficient = 0.50;
    public static final double default_LinearCoefficient = 0.80;
    public static final double slow_TurnCoefficient = 0.40;
    public static final double slow_LinearCoefficient = 0.68;
    public static final double slower_TurnCoefficient = 0.40;
    public static final double slower_LinearCoefficient = 0.45;

    // Auto balance constants
    public static final double kBalanceSetpoint = 0F;
    public static final double adjust_kP = 0.04;
    public static final double adjust_kD = 1;
    public static final double kAdjustOutputLimit = 0.35f;
    public static final double kAdjustLimitThreshold = 0.15f;
    public static final double kBalanceThreshold = 0.2f;
  }

  public static final class VisionConstants {
    public static final double steer_kP = 0.2f;
    public static final double steer_kD = 0.8f;
    public static final double kSteerOutputLimit = 0.35f;
    public static final double kSteerLimitThreshold = 0.15f;

    public static final int kAprilTagPipeline = 0;
    public static final int kReflectiveTapePipeline = 1;
  }

  public static final class GrabberConstants {
    public static final int PCM_CANPort = 20;

    public static final int solenoid1_ForwardChannel = 0;
    public static final int solenoid1_ReverseChannel = 1;
    public static final int solenoid2_ForwardChannel = 2;
    public static final int solenoid2_ReverseChannel = 3;

    public static final int m_GrabberPort = 4;

    public static final double kGrabSpeed = 0.3;
    public static final double kUngrabSpeed = 0.44;
  }

  public static final class ElevatorConstants {
    public static final int m_MainElevatorPort = 1;
    public static final int m_ElevatorRotary1Port = 2;
    public static final int m_ElevatorRotary2Port = 3;

    public static final int m_MainElevatorLimitSwitchDIO = 4;
    // public static final int[] m_MainElevatorEncoderDIO = { 0, 1 };
    // public static final int[] m_ElevatorRotaryEncoderDIO = { 2, 3 };
    public static final double kElevatorTick2Meters = 1.0 / 2048 * 3.25 * 0.04 * Math.PI; // ticks x 1 rotation/PPR x 3.25:1 (gear ratio) x 0.04π m (circumference)/1 rotation = ? meters
    public static final double kElevatorRotaryTick2Deg = 360.0 / 2048;                    // ticks x round angle (360 deg)/PPR x 10.71:1 (toughbox mini + 2 cims gear ratio)

    public static final double e_kP = 1f;
    public static final double e_kI = 0f;
    public static final double e_kD = 4f;
    public static final double e_rotary_kP = 0.5f;
    public static final double e_rotary_kI = 0f;
    public static final double e_rotary_kD = 0.6f;

    public static final double kElevatorSpeedCoeefficient = 0.35;
    public static final double kElevatorRotarySpeedCoeefficient = 0.5;
  }
  
  public static final class OIConstants {
    // Joystick ports
    public static final int kDriverJoystickPort = 0;
    public static final int kSpectreJoystickPort = 1;

    // Joystick button indexes/axes
    public static final int kReverseDriveDirectionButtonIdx = 7;    //driveController
    public static final int kSlowDriveButtonIdx = 5;                //driveController
    public static final int kSlowerDriveButtonIdx = 1;              //driveController
    public static final int kAutoAlignAprilTagButtonIdx = 6;        //driveController
    public static final int kAutoAlignReflectiveTapeButtonIdx = 4;  //driveController
    
    // public static final int kGrabConeButtonIdx = 1;              //spectre
    public static final int kGrabCubeButtonIdx = 5;                 //spectre
    public static final int kUngrabButtonIdx = 6;                   //spectre
    
    public static final int kElevatorUpIdx = 1;                     //spectre     
    public static final int kElevatorDownIdx = 3;                   //spectre     
  
    public static final int kElevatorTopIdx = 2;                    //spectre     
    public static final int kElevatorBottomIdx = 4;                 //spectre     
  }
}
