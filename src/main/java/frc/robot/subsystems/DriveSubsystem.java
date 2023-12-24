// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class DriveSubsystem extends SubsystemBase {
  
  // Left Drivetrain
  private final WPI_VictorSPX m_leftDrivePrimary = new WPI_VictorSPX(DrivetrainConstants.m_LeftDrivePrimaryPort); // m_frontLeft
  private final WPI_VictorSPX m_leftDriveFollower = new WPI_VictorSPX(DrivetrainConstants.m_LeftDriveFollowerPort); // m_rearLeft
  private MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftDriveFollower, m_leftDrivePrimary);

  // Right Drivetrain
  private final WPI_VictorSPX m_rightDrivePrimary = new WPI_VictorSPX(DrivetrainConstants.m_RightDrivePrimaryPort); // m_frontRight
  private final WPI_VictorSPX m_rightDriveFollower = new WPI_VictorSPX(DrivetrainConstants.m_RightDriveFollowerPort); // m_rearRight
  private MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightDriveFollower, m_rightDrivePrimary);

  private DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);

  public AHRS gyro = new AHRS(); //gyro
  private double adjust_output = 0f;
  private double adjust_error = 0.0f;
  private double adjust_last_error = 0.0f;
  private double adjust_error_diff = 0.0f; // adjust error difference = adjust error - adjust last error: for kD

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftDrivePrimary.configFactoryDefault();
    m_leftDriveFollower.configFactoryDefault();
    m_rightDrivePrimary.configFactoryDefault();
    m_rightDriveFollower.configFactoryDefault();

    m_leftDriveFollower.follow(m_leftDrivePrimary);
    m_rightDriveFollower.follow(m_rightDrivePrimary);
    
    m_rightDrive.getInverted();

    m_drive.setDeadband(Constants.kVictorDeadband);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro Yaw:  ", getYawValue());
    SmartDashboard.putNumber("Gyro Pitch:  ", getPitchValue());
  }
  
  public void arcade(double zTurn, double xLinear) {
    m_drive.arcadeDrive(zTurn, xLinear);
  }

  public void stop() {
    arcade(0, 0);
  }

  public double getYawValue() {
    return gyro.getYaw();
  }

  public double getPitchValue() {
    return gyro.getPitch();
  }

  public void brakeMotors(boolean on) {
    NeutralMode mode;
    if (on) {
      mode = NeutralMode.Brake;
    } else
      mode = NeutralMode.Coast;

    m_leftDrivePrimary.setNeutralMode(mode);
    m_leftDriveFollower.setNeutralMode(mode);
    m_rightDrivePrimary.setNeutralMode(mode);
    m_rightDriveFollower.setNeutralMode(mode);
  }

  public double Aim_Balance_Hack(double yaw, double pitch) {
    adjust_last_error = adjust_error;
    adjust_error = DrivetrainConstants.kBalanceSetpoint - pitch;
    adjust_error_diff = adjust_error - adjust_last_error;

    if (Math.abs(adjust_error) > 3f) {
      // adjust_output = adjust_error * adjust_kP + adjust_last_error * adjust_kD;
      adjust_output = adjust_error * DrivetrainConstants.adjust_kP + adjust_error_diff * DrivetrainConstants.adjust_kD;
      if (adjust_output > 0f && adjust_output < DrivetrainConstants.kAdjustOutputLimit) {
        adjust_output = DrivetrainConstants.kAdjustLimitThreshold;
      } else if (adjust_output > -DrivetrainConstants.kAdjustOutputLimit && adjust_output < 0f) {
        adjust_output = -DrivetrainConstants.kAdjustLimitThreshold;
      }
      SmartDashboard.putNumber("adjust_output:  ", adjust_output);
    } else {
      adjust_output = 0.0f;
    }
    
    return adjust_output;
  }
}
