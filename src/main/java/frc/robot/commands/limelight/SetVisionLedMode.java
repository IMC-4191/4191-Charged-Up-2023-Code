// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class SetVisionLedMode extends CommandBase {

  private VisionSubsystem VISION_SUBSYSTEM;
  private int mode;

  /** Creates a new SetVisionLedMode. */
  public SetVisionLedMode(VisionSubsystem limelight, int mode) {
    this.VISION_SUBSYSTEM = limelight;
    this.mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VISION_SUBSYSTEM.setLedMode(mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Vision led mode set to: " + mode);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
