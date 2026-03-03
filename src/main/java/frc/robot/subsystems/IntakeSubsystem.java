// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax leftIntakeMotor;
  private final SparkMax rightIntakeMotor;
  
  public IntakeSubsystem() {
    leftIntakeMotor = new SparkMax(16, MotorType.kBrushless);
    rightIntakeMotor = new SparkMax(17, MotorType.kBrushless);
  }

  public void runIntakePivot(double speed) {
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
