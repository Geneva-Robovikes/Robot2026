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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
  private final PhotonCamera cameraOne;
  private final PhotonCamera cameraTwo;

  private final Transform3d cameraOnePosition;
  private final Transform3d cameraTwoPosition;

  private final PhotonPoseEstimator cameraOnePoseEstimator;
  private final PhotonPoseEstimator cameraTwoPoseEstimator;
  
  public final AprilTagFieldLayout kTagLayout;

  private final CommandSwerveDrivetrain swerveDrivetrain;

  private final VisionSystemSim visionSim;

  public Vision(CommandSwerveDrivetrain swerveDrivetrain) {
    this.swerveDrivetrain = swerveDrivetrain;

    kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    visionSim = new VisionSystemSim("main");

    cameraOne = new PhotonCamera("cameraOne");
    cameraTwo = new PhotonCamera("cameraTwo");  

    cameraOnePosition = new Transform3d(new Translation3d(Units.inchesToMeters(-10.875), Units.inchesToMeters(-10.875), 0), new Rotation3d(Units.degreesToRadians(135), 0, 0));
    cameraTwoPosition = new Transform3d(new Translation3d(Units.inchesToMeters(-10.875), Units.inchesToMeters(10.875), 0), new Rotation3d(Units.degreesToRadians(-45), 0, 0));

    cameraOnePoseEstimator = new PhotonPoseEstimator(kTagLayout, cameraOnePosition);
    cameraTwoPoseEstimator = new PhotonPoseEstimator(kTagLayout, cameraTwoPosition);

    configureSim();
  }

  @Override
  public void periodic() {
    var cameraOneResult = cameraOne.getLatestResult();
    var cameraTwoResult = cameraTwo.getLatestResult();

    Optional<EstimatedRobotPose> cameraOnePose = cameraOnePoseEstimator.estimateCoprocMultiTagPose(cameraOneResult);
    Optional<EstimatedRobotPose> cameraTwoPose = cameraTwoPoseEstimator.estimateCoprocMultiTagPose(cameraTwoResult);

    if (cameraOnePose.isPresent()) {
      swerveDrivetrain.addVisionMeasurement(
        cameraOnePose.get().estimatedPose.toPose2d(), 
        cameraOnePose.get().timestampSeconds);
    }

    if (cameraTwoPose.isPresent()) {
      swerveDrivetrain.addVisionMeasurement(
      cameraTwoPose.get().estimatedPose.toPose2d(), 
      cameraTwoPose.get().timestampSeconds);
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
