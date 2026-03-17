// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MechanismEnum;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final TalonFX intakePivotMotor;

  private MechanismEnum STATE = MechanismEnum.NULL;

  /* Left intake positive, right negative */
  
  public IntakeSubsystem() {
    /* TODO: add feedforward */

    intakeMotor = new TalonFX(13);
    intakePivotMotor = new TalonFX(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Position" , intakePivotMotor.getRotorPosition().getValueAsDouble());

    SmartDashboard.putString("Intake State", STATE.name());

  }

  private void runPivotMotors(MechanismEnum position) {


    switch (position) {
      case INTAKE_UP:
            STATE = MechanismEnum.INTAKE_UP;

            //right negative, left positive
            intakePivotMotor.set(0.4);
            break;
      case INTAKE_DOWN:

            STATE = MechanismEnum.INTAKE_DOWN;
            intakePivotMotor.set(-0.4);
            break;
      default:
        break;
    }
  }

  private void stopIntakePivotMotor() {
    intakePivotMotor.set(0);
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

  public Command stopPivotMotor() {
    return run(() -> stopIntakePivotMotor());
  }

  public Command intake() {
    return run(() -> intakeMotor.set(-.4));
  }

  public Command stopIntaking() {
    return run(() -> intakeMotor.set(0));
  }
}
