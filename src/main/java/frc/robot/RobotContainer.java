// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drivetrain.AutoAlignCommand;
import frc.robot.commands.drivetrain.AutoBalanceCommand;
import frc.robot.commands.drivetrain.DriveWithConstantSpeedCommand;
import frc.robot.commands.drivetrain.DriveWithJoystickCommand;
import frc.robot.commands.elevator_and_rotary.ElevatorConstantSpeedCommand;
// import frc.robot.commands.elevator_and_rotary.ElevatorJoystickCommand;
import frc.robot.commands.elevator_and_rotary.ElevatorRotaryConstantSpeedCommand;
// import frc.robot.commands.elevator_and_rotary.ElevatorRotaryJoystickCommand;
// import frc.robot.commands.elevator_and_rotary.SetElevatorPositionCommand;
// import frc.robot.commands.elevator_and_rotary.SetElevatorRotaryPositionCommand;
// import frc.robot.commands.grabber.GrabConeCommand;
import frc.robot.commands.grabber.GrabCubeCommand;
import frc.robot.commands.grabber.UngrabCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorRotarySubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.DrivetrainConstants;
// import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.OIConstants;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem();
  private VisionSubsystem VISION_SUBSYSTEM = new VisionSubsystem();
  private GrabberSubsystem GRABBER_SUBSYSTEM = new GrabberSubsystem();
  private ElevatorSubsystem ELEVATOR_SUBSYSTEM = new ElevatorSubsystem();
  private ElevatorRotarySubsystem ELEVATOR_ROTARY_SUBSYSTEM = new ElevatorRotarySubsystem();

  private final Joystick driveController = new Joystick(OIConstants.kDriverJoystickPort);
  private final Joystick spectre = new Joystick(OIConstants.kSpectreJoystickPort);
  
  private final JoystickButton SLOW_DRIVE_BUTTON = new JoystickButton(driveController, OIConstants.kSlowDriveButtonIdx);
  private final JoystickButton SLOWER_DRIVE_BUTTON = new JoystickButton(driveController, OIConstants.kSlowerDriveButtonIdx);
  private final JoystickButton AUTO_ALIGN_APRIL_BUTTON = new JoystickButton(driveController, OIConstants.kAutoAlignAprilTagButtonIdx);
  private final JoystickButton AUTO_ALIGN_REFLECTIVE_BUTTON = new JoystickButton(driveController, OIConstants.kAutoAlignReflectiveTapeButtonIdx);
  // private final JoystickButton GRAB_CONE_BUTTON = new JoystickButton(spectre, OIConstants.kGrabConeButtonIdx);
  private final JoystickButton GRAB_CUBE_BUTTON = new JoystickButton(spectre, OIConstants.kGrabCubeButtonIdx);
  private final JoystickButton UNGRAB_BUTTON = new JoystickButton(spectre, OIConstants.kUngrabButtonIdx);
  private final JoystickButton ELEVATOR_DOWN_BUTTON = new JoystickButton(spectre, OIConstants.kElevatorUpIdx);
  private final JoystickButton ELEVATOR_UP_BUTTON = new JoystickButton(spectre, OIConstants.kElevatorDownIdx);
  private final JoystickButton ELEVATOR_TOP_BUTTON = new JoystickButton(spectre, OIConstants.kElevatorTopIdx);
  private final JoystickButton ELEVATOR_BOTTOM_BUTTON = new JoystickButton(spectre, OIConstants.kElevatorBottomIdx);



  // private final DoubleSolenoid solenoid1 = new DoubleSolenoid(
  //     GrabberConstants.PCM_CANPort, PneumaticsModuleType.CTREPCM,
  //     GrabberConstants.solenoid1_ForwardChannel, 
  //     GrabberConstants.solenoid1_ReverseChannel
  // );
  // private final DoubleSolenoid solenoid2 = new DoubleSolenoid(
  //     GrabberConstants.PCM_CANPort, PneumaticsModuleType.CTREPCM, 
  //     GrabberConstants.solenoid2_ForwardChannel,
  //     GrabberConstants.solenoid2_ReverseChannel
  // );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    defaultCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    SLOW_DRIVE_BUTTON.whileTrue(new DriveWithJoystickCommand(
        DRIVE_SUBSYSTEM, () -> driveController.getY(), () -> driveController.getZ(),
        DrivetrainConstants.slow_TurnCoefficient, DrivetrainConstants.slow_LinearCoefficient
    ));
    SLOWER_DRIVE_BUTTON.whileTrue(new DriveWithJoystickCommand(
        DRIVE_SUBSYSTEM, () -> driveController.getY(), () -> driveController.getZ(),
        DrivetrainConstants.slower_TurnCoefficient, DrivetrainConstants.slower_LinearCoefficient
    ));
    AUTO_ALIGN_APRIL_BUTTON.whileTrue(new AutoAlignCommand(
        VISION_SUBSYSTEM, DRIVE_SUBSYSTEM, 0,
        () -> VISION_SUBSYSTEM.tx.getDouble(0.0), () -> VISION_SUBSYSTEM.ty.getDouble(0.0), () -> VISION_SUBSYSTEM.tv.getDouble(0.0),
        () -> VISION_SUBSYSTEM.ta.getDouble(0.0), () -> VISION_SUBSYSTEM.getpipe.getDouble(0.0)
    ));
    AUTO_ALIGN_REFLECTIVE_BUTTON.whileTrue(new AutoAlignCommand(
        VISION_SUBSYSTEM, DRIVE_SUBSYSTEM, 1,
        () -> VISION_SUBSYSTEM.tx.getDouble(0.0), () -> VISION_SUBSYSTEM.ty.getDouble(0.0), () -> VISION_SUBSYSTEM.tv.getDouble(0.0),
        () -> VISION_SUBSYSTEM.ta.getDouble(0.0), () -> VISION_SUBSYSTEM.getpipe.getDouble(0.0)
    ));

    // GRAB_CONE_BUTTON.onTrue(new GrabConeCommand(GRABBER_SUBSYSTEM, solenoid1, solenoid2));
    GRAB_CUBE_BUTTON.whileTrue(new GrabCubeCommand(GRABBER_SUBSYSTEM));
    UNGRAB_BUTTON.whileTrue(new UngrabCommand(GRABBER_SUBSYSTEM));

    ELEVATOR_UP_BUTTON.whileTrue(new ElevatorConstantSpeedCommand(ELEVATOR_SUBSYSTEM, -1));
    ELEVATOR_DOWN_BUTTON.whileTrue(new ElevatorConstantSpeedCommand(ELEVATOR_SUBSYSTEM, 1));

    ELEVATOR_TOP_BUTTON.whileTrue(new ElevatorRotaryConstantSpeedCommand(ELEVATOR_ROTARY_SUBSYSTEM, 1));
    ELEVATOR_BOTTOM_BUTTON.whileTrue(new ElevatorRotaryConstantSpeedCommand(ELEVATOR_ROTARY_SUBSYSTEM, -1));
  }

  private void defaultCommands() {
    DRIVE_SUBSYSTEM.setDefaultCommand(new DriveWithJoystickCommand(DRIVE_SUBSYSTEM, () -> driveController.getY(), () -> driveController.getZ(), DrivetrainConstants.default_TurnCoefficient, DrivetrainConstants.default_LinearCoefficient));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SequentialCommandGroup(
      // new ElevatorRotaryConstantSpeedCommand(ELEVATOR_ROTARY_SUBSYSTEM, 1).withTimeout(.5f),
      // new ElevatorConstantSpeedCommand(ELEVATOR_SUBSYSTEM, 1).withTimeout(.5f),
        // new UngrabCommand(GRABBER_SUBSYSTEM),,
        new ElevatorRotaryConstantSpeedCommand(ELEVATOR_ROTARY_SUBSYSTEM, 1).withTimeout(0.2),
        new ElevatorConstantSpeedCommand(ELEVATOR_SUBSYSTEM, -1).raceWith(new WaitCommand(1.3)),
        new UngrabCommand(GRABBER_SUBSYSTEM).raceWith(new WaitCommand(1.3)),
        new DriveWithConstantSpeedCommand(DRIVE_SUBSYSTEM, 0.85f, 0.8).withTimeout(2f)); //for charging station 7 seconds
    
  }
}
