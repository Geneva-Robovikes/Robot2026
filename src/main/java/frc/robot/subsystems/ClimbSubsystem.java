// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private final TalonFX leftClimbMotor;
  private final TalonFX rightClimbMotor;

  private final MotionMagicVoltage motionMagic;

  //Rotor positions at the bottom and top of the elevator
  private static final double RETRACTED = 0;
  private static final double EXTENDED = 58;


  public ClimbSubsystem() {
    motionMagic = new MotionMagicVoltage(0);
    leftClimbMotor = new TalonFX(14);
    rightClimbMotor = new TalonFX(15);

    leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
    rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 1;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.Slot0.kG = 0.1;
    config.Slot0.kS = 0.1;
    config.Slot0.kV = 0.1;

    config.MotionMagic.MotionMagicAcceleration = 400;
    config.MotionMagic.MotionMagicCruiseVelocity = 200;

    leftClimbMotor.getConfigurator().apply(config);
    rightClimbMotor.getConfigurator().apply(config);

    /*We want the motors to run at the same speed, so this code
    sets the right motor to follow the speed of the left motor*/
    rightClimbMotor.setControl(new Follower(leftClimbMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void goToPosition(double rotations) {
    leftClimbMotor.setControl(motionMagic.withPosition(rotations));
  }

  @Override
  public void periodic() {}

  public Command extend() {
    return run(() -> goToPosition(EXTENDED));
  }

  public Command retract() {
    return run(() -> goToPosition(RETRACTED));
  }
}
