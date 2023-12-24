// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator_and_rotary;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorRotarySubsystem;

public class ElevatorRotaryJoystickCommand extends CommandBase {

  private ElevatorRotarySubsystem ELEVATOR_ROTARY_SUBSYSTEM;
  private final Supplier<Double> speedFunction;
  private double realTimeSpeed;

  /** Creates a new ElevatorRotaryJoystickCommand. */
  public ElevatorRotaryJoystickCommand(ElevatorRotarySubsystem elevatorRotary, Supplier<Double> speedFunction) {
    this.ELEVATOR_ROTARY_SUBSYSTEM = elevatorRotary;
    this.speedFunction = speedFunction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ELEVATOR_ROTARY_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Elevator started rotating.");
    realTimeSpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    realTimeSpeed = speedFunction.get();
    ELEVATOR_ROTARY_SUBSYSTEM.setMotor(realTimeSpeed * ElevatorConstants.kElevatorRotarySpeedCoeefficient);

    // if (ELEVATOR_ROTARY_SUBSYSTEM.getEncoderDegrees() <= 10) {
    //   ELEVATOR_ROTARY_SUBSYSTEM.stop();
    // }
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
