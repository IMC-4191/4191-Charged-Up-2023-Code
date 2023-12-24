// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator_and_rotary;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorPositionCommand extends CommandBase {
  
  private ElevatorSubsystem ELEVATOR_SUBSYSTEM;
  private PIDController pidController;

  double speed;

  /** Creates a new SetElevatorPositionCommand. */
  public SetElevatorPositionCommand(ElevatorSubsystem elevator, double setpoint) {
    this.ELEVATOR_SUBSYSTEM = elevator;
    this.pidController = new PIDController(ElevatorConstants.e_kP, ElevatorConstants.e_kI, ElevatorConstants.e_kP);
    pidController.setSetpoint(setpoint);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ELEVATOR_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Elevator position setting.");
    pidController.reset();

    speed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // speed = pidController.calculate(ELEVATOR_SUBSYSTEM.getEncoderMeters());
    ELEVATOR_SUBSYSTEM.setMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ELEVATOR_SUBSYSTEM.stop();
    System.out.println("Elevator position set!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
