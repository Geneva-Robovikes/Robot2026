// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Ballistics;
import frc.robot.utils.MechanismEnum;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX leftShooterMotor;
  private final TalonFX rightShooterMotor;
  private final TalonFX handoffMotor;

  private final SparkMax agitatorMotor;
  private final SparkMax hoodMotor;

  private final CANcoder hoodEncoder;

  private final PIDController hoodPidController;

  private final Ballistics ballistics;

  private MechanismEnum STATE = MechanismEnum.NULL;

  /* 
   * Handoff: Positive
   * Shooter: Negative
   * Agitator: Positive
   */


  public ShooterSubsystem() {
    leftShooterMotor = new TalonFX(20);
    rightShooterMotor = new TalonFX(21);
    handoffMotor = new TalonFX(24);

    hoodEncoder = new CANcoder(30);

    hoodPidController = new PIDController(.32, 0, 0);

    ballistics = new Ballistics();

    hoodMotor = new SparkMax(22, MotorType.kBrushless);
    agitatorMotor = new SparkMax(23, MotorType.kBrushless);

    rightShooterMotor.setControl(new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", leftShooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hood Angle", hoodEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood PID Controller", hoodPidController.calculate(hoodEncoder.getPosition().getValueAsDouble(), ballistics.calculateShooterAngle()));

    SmartDashboard.putString("Shooter State", STATE.name());
  }

  private void hoodToAngle(double angle) {
    /* TODO:
     * Interpolate CANCoder raw values with real-life angles
     */
    hoodMotor.set(hoodPidController.calculate(hoodEncoder.getPosition().getValueAsDouble(), angle));
  }

  private void shooterHandoffSequence() {
    STATE = MechanismEnum.SHOOTER_SHOOTING;

    double velocity = ballistics.calculateInitialShooterVelocity();
    double angle = ballistics.calculateShooterAngle();

    hoodToAngle(angle);

    handoffMotor.set((-.75*velocity));
    agitatorMotor.set(.35);
    leftShooterMotor.set(velocity);
  }

  public void stopAllMotors() {
    handoffMotor.set(0);
    leftShooterMotor.set(0);
    agitatorMotor.set(0);
    hoodMotor.set(0);

    resetState();
  }

  private void resetState() {
    STATE = MechanismEnum.SHOOTER_IDLE;
  }

  public MechanismEnum getState() {
    return STATE;
  }

  public Command fire() { 
    return run(() -> shooterHandoffSequence());
  }

  public Command testAgitator() {
    return run(() -> agitatorMotor.set(.25));
  }

  public Command testShooter() {
    return run(() -> leftShooterMotor.set(-.25));
  }

  public Command testHandoff() {
    return run(() -> handoffMotor.set(.25));
  }

  public Command stopFiring() {
    return run(() -> stopAllMotors());
  }


}
