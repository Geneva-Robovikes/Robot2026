// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
  private final PhotonCamera cameraOne;
  private final PhotonCamera cameraTwo;
  private final PhotonCamera cameraThree;

  private final Transform3d cameraOnePosition;
  private final Transform3d cameraTwoPosition;
  private final Transform3d cameraThreePosition;

  private final PhotonPoseEstimator cameraOnePoseEstimator;
  private final PhotonPoseEstimator cameraTwoPoseEstimator;
  private final PhotonPoseEstimator cameraThreePoseEstimator;
  
  public final AprilTagFieldLayout kTagLayout;

  private final CommandSwerveDrivetrain swerveDrivetrain;

  private final VisionSystemSim visionSim;

  public Vision(CommandSwerveDrivetrain swerveDrivetrain) {
    this.swerveDrivetrain = swerveDrivetrain;

    kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    visionSim = new VisionSystemSim("main");

    cameraOne = new PhotonCamera("cameraOne");
    cameraTwo = new PhotonCamera("cameraTwo");  
    cameraThree = new PhotonCamera("cameraThree");

    cameraOnePosition = new Transform3d(new Translation3d(Units.inchesToMeters(-10.875), Units.inchesToMeters(-10.875), Units.inchesToMeters(7.25)), new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(-145)));
    cameraTwoPosition = new Transform3d(new Translation3d(Units.inchesToMeters(-10.875), Units.inchesToMeters(10.875), Units.inchesToMeters(7.25)), new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(142)));
    cameraThreePosition = new Transform3d(new Translation3d(Units.inchesToMeters(10.875), Units.inchesToMeters(-10.875), Units.inchesToMeters(7.25)), new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(-40)));

    cameraOnePoseEstimator = new PhotonPoseEstimator(kTagLayout, cameraOnePosition);
    cameraTwoPoseEstimator = new PhotonPoseEstimator(kTagLayout, cameraTwoPosition);
    cameraThreePoseEstimator = new PhotonPoseEstimator(kTagLayout, cameraThreePosition);

    configureSim();
  }

  @Override
  public void periodic() {
    var cameraOneResult = cameraOne.getLatestResult();
    var cameraTwoResult = cameraTwo.getLatestResult();
    var cameraThreeResult = cameraThree.getLatestResult();

    if (cameraOneResult.hasTargets()) {
      SmartDashboard.putBoolean("oneHasTarget", true);
    } else {
      SmartDashboard.putBoolean("oneHasTarget", false);
    }

    if (cameraTwoResult.hasTargets()) {
      SmartDashboard.putBoolean("twoHasTarget", true);
    } else {
      SmartDashboard.putBoolean("twoHasTarget", false);
    }

    if (cameraThreeResult.hasTargets()) {
      SmartDashboard.putBoolean("threeHasTarget", true);
    } else {
      SmartDashboard.putBoolean("threeHasTarget", false);
    }

    Optional<EstimatedRobotPose> cameraOnePose = cameraOnePoseEstimator.estimateCoprocMultiTagPose(cameraOneResult);
    Optional<EstimatedRobotPose> cameraTwoPose = cameraTwoPoseEstimator.estimateCoprocMultiTagPose(cameraTwoResult);
    Optional<EstimatedRobotPose> cameraThreePose = cameraThreePoseEstimator.estimateCoprocMultiTagPose(cameraThreeResult);

    if (cameraOnePose.isPresent()) {
      SmartDashboard.putBoolean("camOnePresent", true);
      swerveDrivetrain.addVisionMeasurement(
        cameraOnePose.get().estimatedPose.toPose2d(), 
        cameraOnePose.get().timestampSeconds);
    } else {
      SmartDashboard.putBoolean("camOnePresent", false);
    }

    if (cameraTwoPose.isPresent()) {
      SmartDashboard.putBoolean("camTwoPresent", true);
      swerveDrivetrain.addVisionMeasurement(
      cameraTwoPose.get().estimatedPose.toPose2d(), 
      cameraTwoPose.get().timestampSeconds);
    } else {
      SmartDashboard.putBoolean("camTwoPresent", false);
    }

    if (cameraThreePose.isPresent()) {
      SmartDashboard.putBoolean("camThreePresent", true);
      swerveDrivetrain.addVisionMeasurement(
      cameraThreePose.get().estimatedPose.toPose2d(), 
      cameraThreePose.get().timestampSeconds);
    } else {
      SmartDashboard.putBoolean("camThreePresent", false);
    }
  }

  public void visionSimPeriodic() {}

  private void configureSim() {
    visionSim.addAprilTags(kTagLayout);

    var cameraOneProp = new SimCameraProperties();
    PhotonCameraSim cameraOneSim;

    cameraOneProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraOneProp.setCalibError(.35, .1);
    cameraOneProp.setFPS(20);
    cameraOneProp.setAvgLatencyMs(40);
    cameraOneProp.setLatencyStdDevMs(15);

    cameraOneSim = new PhotonCameraSim(cameraOne, cameraOneProp);

    visionSim.addCamera(cameraOneSim, cameraOnePosition);

    cameraOneSim.enableDrawWireframe(true);
  }
}
