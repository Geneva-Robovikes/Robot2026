// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.enums.MechanismEnum;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final TalonFX intakePivotMotor;

  private final CANcoder intakeCANcoder;

  private final PIDController pidController;


  private MechanismEnum STATE = MechanismEnum.NULL;

  private double INTAKE_DOWN_POSITION = .489;
  private double INTAKE_UP_POSITION = .814;

  /* Left intake positive, right negative */
  
  public IntakeSubsystem() {
    /* TODO: add feedforward, haha get hacked loser!!!!!! */ 

    intakeMotor = new TalonFX(13);
    intakePivotMotor = new TalonFX(14);

    intakeCANcoder = new CANcoder(30);

    pidController = new PIDController(.9, 0, 0);

    intakePivotMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Relative Position" , intakePivotMotor.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Absolute Position", intakeCANcoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake PID Controller Value Down", pidController.calculate(intakeCANcoder.getPosition().getValueAsDouble(), INTAKE_DOWN_POSITION));
    SmartDashboard.putNumber("Intake PID Controller Value Up", pidController.calculate(intakeCANcoder.getPosition().getValueAsDouble(), INTAKE_UP_POSITION));

    SmartDashboard.putString("Intake State", STATE.name());

  }

  private void runPivotMotors(MechanismEnum position) {


    switch (position) {
      case INTAKE_UP_AGITATE:
            STATE = MechanismEnum.INTAKE_UP;

            //up negative, down positive
            //double speedup = -pidController.calculate(intakeCANcoder.getPosition().getValueAsDouble(), INTAKE_UP_POSITION);
            intakePivotMotor.set(-.15);
            intakeMotor.set(-.9);
            break;
      case INTAKE_UP:
            STATE = MechanismEnum.INTAKE_UP;

            //up negative, down positive
            //double speedup = -pidController.calculate(intakeCANcoder.getPosition().getValueAsDouble(), INTAKE_UP_POSITION);
            intakePivotMotor.set(-.15);
            break;
      case INTAKE_DOWN_AUTO:
            STATE = MechanismEnum.INTAKE_DOWN;
            double speeddown = -pidController.calculate(intakeCANcoder.getPosition().getValueAsDouble(), INTAKE_DOWN_POSITION);

            intakePivotMotor.set(speeddown);
            intakeMotor.set(-.9);
            break;
      case INTAKE_DOWN:
            STATE = MechanismEnum.INTAKE_DOWN;

            intakePivotMotor.set(.1);

      default:
        break;
    }
  }

  private void stopIntakePivotMotor() {
    intakePivotMotor.set(0);
    intakeMotor.set(0);
  }

  public MechanismEnum getState() {
    return STATE;
  }


  public Command extend_auto() {
    return run(() -> runPivotMotors(MechanismEnum.INTAKE_DOWN_AUTO));
  }

  public Command extend() {
    return run(() -> runPivotMotors(MechanismEnum.INTAKE_DOWN));
  }

  public Command agitate() {
    return run(() -> runPivotMotors(MechanismEnum.INTAKE_UP_AGITATE));
  }

  public Command retract() {

    return run(() -> runPivotMotors(MechanismEnum.INTAKE_UP));
  }

  public Command stopPivotMotor() {
    return run(() -> stopIntakePivotMotor());
  }

  public Command intake() {
    return run(() -> intakeMotor.set(-1));
  }

  public Command stopIntaking() {
    return run(() -> intakeMotor.set(0));
  }

  public Command outtake() {
    return run(() -> intakeMotor.set(.6));
  }
}
