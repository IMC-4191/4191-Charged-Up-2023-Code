// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorRotarySubsystem extends SubsystemBase {

  // Rotary Motor
  private final WPI_TalonSRX m_elevatorRotary1 = new WPI_TalonSRX(ElevatorConstants.m_ElevatorRotary1Port);
  private final WPI_TalonSRX m_elevatorRotary2 = new WPI_TalonSRX(ElevatorConstants.m_ElevatorRotary2Port);
  private MotorControllerGroup m_elevatorRotary = new MotorControllerGroup(m_elevatorRotary1, m_elevatorRotary2);

  // Rotary Encoder
  // private final Encoder m_elevatorRotaryEncoder = new Encoder(
  //     ElevatorConstants.m_ElevatorRotaryEncoderDIO[0], ElevatorConstants.m_ElevatorRotaryEncoderDIO[1],
  //     false,
  //     EncodingType.k4X);

  /** Creates a new ElevatorRotarySubsystem. */
  public ElevatorRotarySubsystem() {
    m_elevatorRotary1.configFactoryDefault();
    m_elevatorRotary2.configFactoryDefault();
    // m_elevatorRotary.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    
    m_elevatorRotary2.follow(m_elevatorRotary1);

    // m_elevatorRotary.setSensorPhase(false);
    
    // m_elevatorRotaryEncoder.reset();
    // m_elevatorRotary.setSelectedSensorPosition(0, 0, 10);

    // set encoder boundary limits: to stop motors
    // m_elevatorRotary.configReverseSoftLimitThreshold((int) (0 / ElevatorConstants.kElevatorRotaryTick2Deg), 10);
    // m_elevatorRotary.configForwardSoftLimitThreshold((int) (70 / ElevatorConstants.kElevatorRotaryTick2Deg), 10);
    // m_elevatorRotary.configReverseSoftLimitEnable(true, 10);
    // m_elevatorRotary.configForwardSoftLimitEnable(true, 10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // put encoder values for elevator rotary in degrees
    // SmartDashboard.putNumber("ElevatorRotary Encoder Value (deg):  ", getEncoderDegrees());
  }
  
  // public double getEncoderDegrees() {
  //   return m_elevatorRotary.getSelectedSensorPosition() * ElevatorConstants.kElevatorRotaryTick2Deg;
  // }

  public void setMotor(double speed) {
    m_elevatorRotary.set(speed);
  }

  public void stop() {
    m_elevatorRotary.set(0);
  }
  
  public void brakeMotors(boolean on) {
    NeutralMode mode;
    if (on) {
      mode = NeutralMode.Brake;
    } else
      mode = NeutralMode.Coast;

    m_elevatorRotary1.setNeutralMode(mode);
    m_elevatorRotary2.setNeutralMode(mode);
  }
}
