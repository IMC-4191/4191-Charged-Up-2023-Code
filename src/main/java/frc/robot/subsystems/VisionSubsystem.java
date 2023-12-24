// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public NetworkTableEntry tx = table.getEntry("tx");
  public NetworkTableEntry ty = table.getEntry("ty");
  public NetworkTableEntry tv = table.getEntry("tv");
  public NetworkTableEntry ta = table.getEntry("ta");
  public NetworkTableEntry getpipe = table.getEntry("getpipe");

  private boolean hasValidTarget = false;
  private double steer_output = 0.0f;
  private double steer_error = 0.0f;
  private double steer_last_error = 0.0f;
  private double steer_error_diff = 0.0f; // steer error difference = steer error - steer last error: for kD

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double Aim_Goal_Hack(double tv, double tx) {
    if (tv < 1.0) {
      hasValidTarget = false;
      steer_output = 0.0f;
    } else {
      hasValidTarget = true;
      steer_last_error = steer_error;
      steer_error = tx;
      steer_error_diff = steer_error - steer_last_error;
    }

    if (hasValidTarget) {
      if (Math.abs(steer_error) > 1f) {
        // steer_output = steer_error * steer_kP + steer_error_diff * steer_kD;
        steer_output = steer_error * VisionConstants.steer_kP + steer_error_diff * VisionConstants.steer_kD;
        if (steer_output > 0f && steer_output < VisionConstants.kSteerOutputLimit) {
          steer_output = VisionConstants.kSteerLimitThreshold;
        } else if (steer_output > -VisionConstants.kSteerOutputLimit && steer_output < 0f) {
          steer_output = -VisionConstants.kSteerLimitThreshold;
        }
        SmartDashboard.putNumber("steer_output:  ", steer_output);
      } else {
        steer_output = 0.0f;
      }
    }

    return steer_output;
  }

  public void setCamMode(int mode) {
    table.getEntry("camMode").setNumber(mode);
  }

  public void setLedMode(int mode) {
    table.getEntry("ledMode").setNumber(mode);
  }

  public void selectPipeline(double pipe) {
    table.getEntry("pipeline").setNumber(pipe);
  }
}
