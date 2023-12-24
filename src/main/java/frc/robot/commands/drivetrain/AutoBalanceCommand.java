// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {

  private DriveSubsystem DRIVE_SUBSYSTEM;
  
  private Supplier<Double> yawFunction, pitchFunction;
  private double realTimeYaw;
  private double realTimePitch;

  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(DriveSubsystem drivetrain, Supplier<Double> yawFunction, Supplier<Double> pitchFunction) {
    this.DRIVE_SUBSYSTEM = drivetrain;
    this.yawFunction = yawFunction;
    this.pitchFunction = pitchFunction;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Auto balancing on charging station.");
    realTimePitch = 0f;
    realTimeYaw = 0f;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    realTimePitch = pitchFunction.get();
    realTimeYaw = yawFunction.get();

    DRIVE_SUBSYSTEM.arcade(0, DRIVE_SUBSYSTEM.Aim_Balance_Hack(realTimeYaw, realTimePitch));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Balanced on charging station!");
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (realTimePitch > -DrivetrainConstants.kBalanceThreshold && realTimePitch < DrivetrainConstants.kBalanceThreshold)
      return true;
    return false;
  }
}
