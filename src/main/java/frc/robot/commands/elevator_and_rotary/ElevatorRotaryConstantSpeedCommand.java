// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator_and_rotary;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorRotarySubsystem;

public class ElevatorRotaryConstantSpeedCommand extends CommandBase {

  private ElevatorRotarySubsystem ELEVATOR_ROTARY_SUBSYSTEM;
  private double sign;

  /** Creates a new ElevatorRotaryConstantSpeed. */
  public ElevatorRotaryConstantSpeedCommand(ElevatorRotarySubsystem elevatorRotary, double sign) {
    this.ELEVATOR_ROTARY_SUBSYSTEM = elevatorRotary;
    this.sign = sign;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ELEVATOR_ROTARY_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Elevator started rotating.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ELEVATOR_ROTARY_SUBSYSTEM.setMotor(ElevatorConstants.kElevatorRotarySpeedCoeefficient * sign);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Elevator rotated!");
    ELEVATOR_ROTARY_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
