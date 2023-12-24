// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator_and_rotary;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorRotarySubsystem;

public class SetElevatorRotaryPositionCommand extends CommandBase {

  private ElevatorRotarySubsystem ELEVATOR_ROTARY_SUBSYSTEM;
  private PIDController pidController;
  
  double speed;

  /** Creates a new SetElevatorRotaryPositionCommand. */
  public SetElevatorRotaryPositionCommand(ElevatorRotarySubsystem elevatorRotary, double setpoint) {
    this.ELEVATOR_ROTARY_SUBSYSTEM = elevatorRotary;
    this.pidController = new PIDController(ElevatorConstants.e_rotary_kP, ElevatorConstants.e_rotary_kI, ElevatorConstants.e_rotary_kP);
    pidController.setSetpoint(setpoint);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ELEVATOR_ROTARY_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Elevator rotary position setting.");
    pidController.reset();
    
    speed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // speed = pidController.calculate(ELEVATOR_ROTARY_SUBSYSTEM.getEncoderDegrees());
    ELEVATOR_ROTARY_SUBSYSTEM.setMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ELEVATOR_ROTARY_SUBSYSTEM.stop();
    System.out.println("Elevator rotary position set!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
