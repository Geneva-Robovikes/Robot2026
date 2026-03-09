// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.AutoChooser;
import frc.robot.utils.Pathfind;
import frc.robot.utils.Vision;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.07) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle pointAt = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(7, 0, 0)
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.07)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    /* Utils */
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Vision vision = new Vision(drivetrain);
    private final Pathfind pathfind = new Pathfind();
    private final AutoChooser chooser = new AutoChooser();

    /* Subsystems */
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    /* Other */
    private final FollowPath.Builder pathBuilder;
    private final SendableChooser<Path> autoChooser = chooser.getAutoChooser();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public RobotContainer() {
        
        /* BLINE Path Builder */        
        pathBuilder = new FollowPath.Builder(
            drivetrain,                      // The drive subsystem to require
            () -> drivetrain.getState().Pose,             // Supplier for current robot pose
            () -> drivetrain.getState().Speeds,    // Supplier for current speeds
            (speeds) -> drivetrain.setControl(autoRequest.withSpeeds(speeds)),               // Consumer to drive the robot
            new PIDController(3.45,0,0.05),// Translation PID
            new PIDController(3.0, 0.0, 0.0),    // Rotation PID
            new PIDController(1.5, 0.0, 0.0)     // Cross-track PID
        ).withDefaultShouldFlip()                // Auto-flip for red alliance
        .withPoseReset(drivetrain::resetPose);  // Reset odometry at path start

        SmartDashboard.putData("BLINE AutoChooser", autoChooser);
    

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
         //   point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //));

        joystick.x().whileTrue(
            drivetrain.applyRequest(() -> {
                double robotX = drivetrain.getState().Pose.getX();
                double robotY = drivetrain.getState().Pose.getY();

                double targetX = 4.583;
                double targetY = 4.033;

                double dx = targetX - robotX;
                double dy = targetY - robotY;

                Rotation2d targetRotation = new Rotation2d(Math.atan2(dy, dx));

                return pointAt
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withTargetDirection(targetRotation);
            })
        );

        /* While the y button is pressed, pathfind to the tower and extend the climb. After its released, retract the climb. */
        joystick.a().whileTrue(shooterSubsystem.fire()).onFalse(shooterSubsystem.stopFiring());
        joystick.y().whileTrue(intakeSubsystem.intake()).onFalse(intakeSubsystem.stopIntaking());

        joystick.rightTrigger().whileTrue(intakeSubsystem.extend()).onFalse(intakeSubsystem.retract());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); 
        */

        // Reset the field-centric heading on left bumper press.
        joystick.leftTrigger().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Optional<Command> getAutonomousCommand() {
        try {
            Command path = pathBuilder.build(autoChooser.getSelected());

            return Optional.of(path);
        } catch (NullPointerException exception) {
            return Optional.empty();
        }
    }
}
