// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  // Elevator Motor
  private final WPI_TalonSRX m_mainElevator = new WPI_TalonSRX(ElevatorConstants.m_MainElevatorPort);
  
  // Elevator Encoder
  // private final Encoder m_mainElevatorEncoder = new Encoder(
  //     ElevatorConstants.m_MainElevatorEncoderDIO[0], ElevatorConstants.m_MainElevatorEncoderDIO[1],
  //     false,
  //     EncodingType.k4X);  

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_mainElevator.configFactoryDefault();
    
    // m_mainElevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    // m_mainElevator.setSensorPhase(true);
    
    // m_mainElevatorEncoder.reset();
    // m_mainElevator.setSelectedSensorPosition(0, 0, 10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // put encoder values for elevator in meters
    // SmartDashboard.putNumber("Elevator Encoder Value (m):  ", getEncoderMeters());
  }
  
  // public double getEncoderMeters() {
  //   return m_mainElevator.getSelectedSensorPosition() * ElevatorConstants.kElevatorTick2Meters;
  // }
  
  public void setMotor(double speed) {
    m_mainElevator.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    m_mainElevator.set(0);
  }

  public void brakeMotors(boolean on) {
    NeutralMode mode;
    if (on) {
      mode = NeutralMode.Brake;
    } else
      mode = NeutralMode.Coast;

    m_mainElevator.setNeutralMode(mode);
  }
}
