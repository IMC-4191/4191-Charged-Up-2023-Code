// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator_and_rotary;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorConstantSpeedCommand extends CommandBase {

  private ElevatorSubsystem ELEVATOR_SUBSYSTEM;
  private double sign;

  /** Creates a new ElevatorConstantSpeedCommand. */
  public ElevatorConstantSpeedCommand(ElevatorSubsystem elevator, double sign) {
    this.ELEVATOR_SUBSYSTEM = elevator;
    this.sign = sign;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ELEVATOR_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Elevator started running!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ELEVATOR_SUBSYSTEM.setMotor(ElevatorConstants.kElevatorSpeedCoeefficient * sign);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Elevator stopped.");
    ELEVATOR_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
