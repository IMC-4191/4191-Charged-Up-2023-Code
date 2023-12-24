// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWithConstantSpeedCommand extends CommandBase {

  private DriveSubsystem DRIVE_SUBSYSTEM;

  private double linearCoeefficient;

  private double linearSpeed;

  // makes inputs smooth
  // SlewRateLimiter(num) -> can only increase by num in a single second
  SlewRateLimiter linear_limiter = new SlewRateLimiter(DrivetrainConstants.linear_SlewRateLimit);

  /** Creates a new DriveWithConstantSpeedCommand. */
  public DriveWithConstantSpeedCommand(DriveSubsystem drivetrain, double linearSpd, double linearCooef) {
    this.DRIVE_SUBSYSTEM = drivetrain;
    this.linearSpeed = linearSpd;
    this.linearCoeefficient = linearCooef;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DRIVE_SUBSYSTEM.arcade(0, linearSpeed * linearCoeefficient);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
