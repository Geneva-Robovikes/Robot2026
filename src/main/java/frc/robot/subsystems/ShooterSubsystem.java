// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Ballistics;
import frc.robot.utils.enums.MechanismEnum;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX leftShooterMotor;
  private final TalonFX rightShooterMotor;
  private final TalonFX handoffMotor;

  private final SparkMax agitatorMotor;
  private final SparkMax hoodMotor;

  //private final CANcoder hoodEncoder;

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


  public ShooterSubsystem(Ballistics ballistics) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    /* TODO: tune */

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.112;
    config.Slot0.kA = 0.0;

    config.Slot0.kP = 0.6;
    //config.Slot0.kI = 0.0;
    //config.Slot0.kD = 0.0;

    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    leftShooterMotor = new TalonFX(20);
    rightShooterMotor = new TalonFX(21);
    handoffMotor = new TalonFX(24);

    //hoodEncoder = new CANcoder(30);

    hoodPidController = new PIDController(.001, 0, 0);

    this.ballistics = ballistics;

    hoodMotor = new SparkMax(22, MotorType.kBrushless);
    agitatorMotor = new SparkMax(23, MotorType.kBrushless);

    leftShooterMotor.getConfigurator().apply(config);
    rightShooterMotor.getConfigurator().apply(config);
    handoffMotor.getConfigurator().apply(config);

    rightShooterMotor.setControl(new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Actual Speed RPM", leftShooterMotor.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Shooter Requested Speed RPM", ballistics.calculateInitialShooterRPM());
    SmartDashboard.putNumber("Handoff Speed RPM", handoffMotor.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putBoolean("Shooter At Speed", atSpeed());
    SmartDashboard.putNumber("Hood Angle", hoodMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Hood PID Controller", hoodPidController.calculate(hoodMotor.getEncoder().getPosition(), ballistics.calculateShooterAngle()));

    SmartDashboard.putString("Shooter State", STATE.name());
  }

  private void shooterToRPM(double rpm) {
    targetRPS = rpm/60;
    
    leftShooterMotor.setControl(velocityRequest.withVelocity(targetRPS));
  }

  
  private void handoffToRPM(double rpm) {
    targetRPS = rpm/60;
    
    handoffMotor.setControl(velocityRequest.withVelocity(targetRPS));
  }

  private void shooterHandoffSequence() {
    STATE = MechanismEnum.SHOOTER_CHARGING;
    shooterToRPM(ballistics.calculateInitialShooterRPM());
    //hoodToAngle(-2.023);




    if (atSpeed()) {
      handoffToRPM(ballistics.calculateInitialHandoffRPM());
      agitatorMotor.set(.85);
      hoodMotor.set(.12);

      STATE = MechanismEnum.SHOOTER_SHOOTING;
    } else {
      //handoffMotor.set(0);
      //agitatorMotor.set(0);
      hoodMotor.set(0);

      STATE = MechanismEnum.SHOOTER_CHARGING;
    }
  }

  private double getShooterRPM() {
    return leftShooterMotor.getVelocity().getValueAsDouble() * 60;
  }

  private boolean atSpeed() {
    double error = Math.abs(Math.abs(getShooterRPM()) - Math.abs(targetRPS) * 60);

    SmartDashboard.putNumber("Shooter Error", error);
    return error < 100;
  }

  public void stopAllMotors() {
    handoffMotor.stopMotor();
    leftShooterMotor.stopMotor();
    agitatorMotor.stopMotor();
    hoodMotor.stopMotor();

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

  public Command reverseAgitator() {
    return run(() -> agitatorMotor.set(-.5));
  } 

  public Command hoodUp() {
    return run(() -> hoodMotor.set(-.25));
  }

  public Command hoodDown() {
    return run(() -> hoodMotor.set(.25));
  }


}
