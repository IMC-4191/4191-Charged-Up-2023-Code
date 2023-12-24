// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignCommand extends CommandBase {

  private VisionSubsystem VISION_SUBSYSTEM;
  private DriveSubsystem DRIVE_SUBSYSTEM;

  private Supplier<Double> xFunction, yFunction, vFunction, areaFunction, pipelineFunction;
  private double realTimeX;
  private double realTimeY;
  private double realTimeV;
  private double realTimeArea;
  private double realTimePipeline;
  private double targetPipeline;

  /** Creates a new AutoAlignAprilTagCommand. */
  public AutoAlignCommand(
      VisionSubsystem limelight, DriveSubsystem drivetrain, int pipeL,
      Supplier<Double> xFunction, Supplier<Double> yFunction,
      Supplier<Double> vFunction, Supplier<Double> areaFunction,
      Supplier<Double> pipelineFunction
  ) {
    this.VISION_SUBSYSTEM = limelight;
    this.DRIVE_SUBSYSTEM = drivetrain;
    this.xFunction = xFunction;
    this.yFunction = yFunction;
    this.vFunction = vFunction;
    this.areaFunction = areaFunction;
    this.pipelineFunction = pipelineFunction;
    this.targetPipeline = pipeL;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(VISION_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Aligning to grid.");
    VISION_SUBSYSTEM.setCamMode(0);
    VISION_SUBSYSTEM.setLedMode(0);
    VISION_SUBSYSTEM.selectPipeline(targetPipeline);
    realTimeX = 0f;
    realTimeY = 0f;
    realTimeV = 0f;
    realTimeArea = 0f;
    realTimePipeline = 0f;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    realTimeX = xFunction.get();
    realTimeY = yFunction.get();
    realTimeV = vFunction.get();
    realTimeArea = areaFunction.get();
    realTimePipeline = pipelineFunction.get();

    SmartDashboard.putNumber("LimelightX:  ", realTimeX);
    SmartDashboard.putNumber("LimelightY:  ", realTimeY);
    SmartDashboard.putNumber("LimelightV:  ", realTimeV);
    SmartDashboard.putNumber("LimelightArea  ", realTimeArea);
    SmartDashboard.putNumber("Pipeline:  ", realTimePipeline);

    if (Math.abs(realTimeX) > 0.4) {
      DRIVE_SUBSYSTEM.arcade(VISION_SUBSYSTEM.Aim_Goal_Hack(realTimeV, realTimeX), 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Auto align stopped.");
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
