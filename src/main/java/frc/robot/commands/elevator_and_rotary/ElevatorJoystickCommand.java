// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator_and_rotary;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorJoystickCommand extends CommandBase {

  private ElevatorSubsystem ELEVATOR_SUBSYSTEM;
  private final Supplier<Double> speedFunction;
  private double realTimeSpeed;
  
  /** Creates a new ElevatorJoystickCommand. */
  public ElevatorJoystickCommand(ElevatorSubsystem elevator, Supplier<Double> speedFunction) {
    this.ELEVATOR_SUBSYSTEM = elevator;
    this.speedFunction = speedFunction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ELEVATOR_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Elevator started running!");
    realTimeSpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    realTimeSpeed = speedFunction.get();
    ELEVATOR_SUBSYSTEM.setMotor(realTimeSpeed * ElevatorConstants.kElevatorSpeedCoeefficient);

    // if (ELEVATOR_SUBSYSTEM.getEncoderMeters() <= 0.1) {
    //   ELEVATOR_SUBSYSTEM.stop();
    // }
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
