// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {

  // private final Compressor compressor = new Compressor(GrabberConstants.PCM_CANPort, PneumaticsModuleType.CTREPCM);

  private final WPI_TalonSRX m_grabber = new WPI_TalonSRX(GrabberConstants.m_GrabberPort); 
  
  /** Creates a new PneumaticsSubsystem. */
  public GrabberSubsystem() {
    m_grabber.configFactoryDefault();
  }
  
  public void setMotor(double speed) {
    m_grabber.set(ControlMode.PercentOutput, speed);
  }
  
  public void stop() {
    m_grabber.set(0);
  }

  // public void actuate(DoubleSolenoid sol) {
  //   sol.set(DoubleSolenoid.Value.kForward);
  // }
  
  // public void deactuate(DoubleSolenoid sol) {
  //   sol.set(DoubleSolenoid.Value.kReverse);
  // }

  // public void disableCompressor() {
  //   compressor.disable();
  // }

  // public void enableCompressor() {
  //   compressor.enableDigital();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
