// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

  private double targetRPS = 0;

  private final VelocityVoltage velocityRequest =
    new VelocityVoltage(0).withSlot(0).withEnableFOC(false);

  /* 
   * Handoff: Positive
   * Shooter: Negative
   * Agitator: Positive
   */


  public ShooterSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    /* TODO: tune */

    config.Slot0.kS = 0.18;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.01;

    config.Slot0.kP = 0.001;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    leftShooterMotor = new TalonFX(20);
    rightShooterMotor = new TalonFX(21);
    handoffMotor = new TalonFX(24);

    hoodEncoder = new CANcoder(30);

    hoodPidController = new PIDController(.32, 0, 0);

    ballistics = new Ballistics();

    hoodMotor = new SparkMax(22, MotorType.kBrushless);
    agitatorMotor = new SparkMax(23, MotorType.kBrushless);

    leftShooterMotor.getConfigurator().apply(config);
    rightShooterMotor.getConfigurator().apply(config);

    rightShooterMotor.setControl(new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed RPM", leftShooterMotor.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putBoolean("Shooter At Speed", atSpeed());
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

  private void shooterToRPM(double rpm) {
    targetRPS = rpm/60;
    
    leftShooterMotor.setControl(velocityRequest.withVelocity(targetRPS));
  }

  private void shooterHandoffSequence() {
    STATE = MechanismEnum.SHOOTER_CHARGING;
    shooterToRPM(ballistics.calculateInitialShooterRPM());
    hoodToAngle(ballistics.calculateShooterAngle());


    agitatorMotor.set(.65);

    if (atSpeed()) {
      handoffMotor.set(.55);
          
      STATE = MechanismEnum.SHOOTER_SHOOTING;
    } else {
      handoffMotor.set(0);

      STATE = MechanismEnum.SHOOTER_CHARGING;
    }
  }

  private void runShooter() {
    shooterToRPM(ballistics.calculateInitialShooterRPM());
  }

  private double getShooterRPM() {
    return leftShooterMotor.getVelocity().getValueAsDouble() * 60;
  }

  private boolean atSpeed() {
    double error = Math.abs(Math.abs(getShooterRPM()) - Math.abs(targetRPS) * 60);

    SmartDashboard.putNumber("Shooter Error", error);
    return error < 500;
  }

  public void stopAllMotors() {
    handoffMotor.stopMotor();
    leftShooterMotor.stopMotor();
    agitatorMotor.stopMotor();

    STATE = MechanismEnum.SHOOTER_IDLE;
  }



  public MechanismEnum getState() {
    return STATE;
  }

  public Command fire() { 
    return run(() -> shooterHandoffSequence());
  }

  public Command stopFiring() {
    return run(() -> stopAllMotors());
  }

  public Command testShooter() {
    return run(() -> runShooter());
  }


}
