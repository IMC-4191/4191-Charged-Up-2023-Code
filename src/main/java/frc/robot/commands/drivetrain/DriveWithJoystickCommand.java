// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWithJoystickCommand extends CommandBase {

  private DriveSubsystem DRIVE_SUBSYSTEM;

  private double turnCoeefficient;
  private double linearCoeefficient;

  private final Supplier<Double> linearFunction, turnFunction;
  private double realTimeLinear;
  private double realTimeTurn;

  // makes inputs smooth
  // SlewRateLimiter(num) -> can only increase by num in a single second
  SlewRateLimiter linear_limiter = new SlewRateLimiter(DrivetrainConstants.linear_SlewRateLimit);
  SlewRateLimiter turn_limiter = new SlewRateLimiter(DrivetrainConstants.turn_SlewRateLimit);

  /** Creates a new DefaultDriveWithJoystickCommand. */
  public DriveWithJoystickCommand(DriveSubsystem drivetrain, Supplier<Double> linearFunction, Supplier<Double> turnFunction, double turnCoeef, double linearCooef) {
    this.DRIVE_SUBSYSTEM = drivetrain;
    this.linearFunction = linearFunction;
    this.turnFunction = turnFunction;
    this.turnCoeefficient = turnCoeef;
    this.linearCoeefficient = linearCooef;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    realTimeLinear = 0;
    realTimeTurn = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    realTimeLinear = linearFunction.get();
    realTimeTurn = turnFunction.get();

    DRIVE_SUBSYSTEM.arcade(realTimeTurn * turnCoeefficient, realTimeLinear * linearCoeefficient);
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
