// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MechanismEnum;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax leftIntakeMotor;
  private final SparkMax rightIntakeMotor;

  private final TalonFX intakeMotor;

  private final PIDController pivotPidController;

  private final double INTAKE_UP = .33;
  private final double INTAKE_DOWN = 0.0;

  private MechanismEnum STATE = MechanismEnum.NULL;

  /* Left intake positive, right negative */
  
  public IntakeSubsystem() {
    leftIntakeMotor = new SparkMax(16, MotorType.kBrushless);
    rightIntakeMotor = new SparkMax(17, MotorType.kBrushless);

    pivotPidController = new PIDController(.6, 0, 0);

    intakeMotor = new TalonFX(13);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Position" , leftIntakeMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake PID", pivotPidController.calculate(leftIntakeMotor.getEncoder().getPosition(), INTAKE_UP));

    SmartDashboard.putString("Intake State", STATE.name());

  }

  private void runPivotMotors(MechanismEnum position) {
    double intakePIDValue = 0;

    switch (position) {
      case INTAKE_UP:
            intakePIDValue = pivotPidController.calculate(leftIntakeMotor.getEncoder().getPosition(), INTAKE_UP);
            STATE = MechanismEnum.INTAKE_UP;

            leftIntakeMotor.set(intakePIDValue);
            rightIntakeMotor.set(-intakePIDValue);

            break;
      case INTAKE_DOWN:
            intakePIDValue = pivotPidController.calculate(leftIntakeMotor.getEncoder().getPosition(), INTAKE_DOWN);
            STATE = MechanismEnum.INTAKE_DOWN;

            leftIntakeMotor.set(intakePIDValue);
            rightIntakeMotor.set(-intakePIDValue);

            break;
      default:
        break;
    }
  }

  public MechanismEnum getState() {
    return STATE;
  }

  public Command extend() {
    return run(() -> runPivotMotors(MechanismEnum.INTAKE_DOWN));
  }

  public Command retract() {
    return run(() -> runPivotMotors(MechanismEnum.INTAKE_UP));
  }

  public Command intake() {
    return run(() -> intakeMotor.set(-.65));
  }
}
