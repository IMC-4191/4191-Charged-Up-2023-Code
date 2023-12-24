// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabConeCommand extends CommandBase {
  
  private GrabberSubsystem GRABBER_SUBSYSTEM;
  private DoubleSolenoid sol1;
  private DoubleSolenoid sol2; 

  /** Creates a new GrabCommand. */
  public GrabConeCommand(GrabberSubsystem grabber, DoubleSolenoid dsol1, DoubleSolenoid dsol2) {
    this.GRABBER_SUBSYSTEM = grabber;
    this.sol1 = dsol1;
    this.sol2 = dsol2;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(GRABBER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // GRABBER_SUBSYSTEM.actuate(sol1);
    // GRABBER_SUBSYSTEM.actuate(sol2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Cone grabbed!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
