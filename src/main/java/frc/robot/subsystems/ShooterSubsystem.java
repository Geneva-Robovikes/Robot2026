// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX leftShooterMotor;
  private final TalonFX rightShooterMotor;
  private final TalonFX handoffMotor;

  private final SparkMax agitatorMotor;
  private final SparkMax hoodMotor;



  public ShooterSubsystem() {
    leftShooterMotor = new TalonFX(20);
    rightShooterMotor = new TalonFX(21);
    handoffMotor = new TalonFX(24);

    hoodMotor = new SparkMax(22, MotorType.kBrushless);
    agitatorMotor = new SparkMax(23, MotorType.kBrushless);

    rightShooterMotor.setControl(new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAgitatorMotorSpeed(double speed) {
    agitatorMotor.set(speed);
  }

  public void setHandoffMotorSpeed(double speed) {
    handoffMotor.set(speed);
  }

  public void setHoodMotorSpeed(double speed) {
    hoodMotor.set(speed);
  }

  public void setShooterMotorSpeed(double speed) {
    leftShooterMotor.set(speed);
  }
}
