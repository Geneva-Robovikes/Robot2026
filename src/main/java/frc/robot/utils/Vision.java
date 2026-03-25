// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
  private final PhotonCamera cameraOne;
  private final PhotonCamera cameraTwo;
  private final PhotonCamera cameraThree;
  //private final PhotonCamera cameraFour;

  //private final PhotonCamera fuelCameraOne;

  private final Transform3d cameraOnePosition;
  private final Transform3d cameraTwoPosition;
  private final Transform3d cameraThreePosition;
  //private final Transform3d cameraFourPosition;

  private final PhotonPoseEstimator cameraOnePoseEstimator;
  private final PhotonPoseEstimator cameraTwoPoseEstimator;
  private final PhotonPoseEstimator cameraThreePoseEstimator;
  //private final PhotonPoseEstimator cameraFourPoseEstimator;
  
  public final AprilTagFieldLayout kTagLayout;

  private final CommandSwerveDrivetrain swerveDrivetrain;

  public Vision(CommandSwerveDrivetrain swerveDrivetrain) {
    this.swerveDrivetrain = swerveDrivetrain;

    kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    cameraOne = new PhotonCamera("cameraOne");
    cameraTwo = new PhotonCamera("cameraTwo");  
    cameraThree = new PhotonCamera("cameraThree");
    //cameraFour = new PhotonCamera("cameraFour");

    //fuelCameraOne = new PhotonCamera("fuelCameraOne");

    cameraThreePosition = new Transform3d(new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(-10.875), Units.inchesToMeters(27.5)), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-90)));
    cameraTwoPosition = new Transform3d(new Translation3d(Units.inchesToMeters(-13), Units.inchesToMeters(14.875), Units.inchesToMeters(27.5)), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(90)));
    cameraOnePosition = new Transform3d(new Translation3d(Units.inchesToMeters(-10.875), Units.inchesToMeters(-19.875), Units.inchesToMeters(7.25)), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)));
    //cameraFourPosition = new Transform3d(new Translation3d(Units.inchesToMeters(10.875), Units.inchesToMeters(-10.875), Units.inchesToMeters(7.25)), new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(-40)));

    cameraOnePoseEstimator = new PhotonPoseEstimator(kTagLayout, cameraOnePosition);
    cameraTwoPoseEstimator = new PhotonPoseEstimator(kTagLayout, cameraTwoPosition);
    cameraThreePoseEstimator = new PhotonPoseEstimator(kTagLayout, cameraThreePosition);
    //cameraFourPoseEstimator = new PhotonPoseEstimator(kTagLayout, cameraFourPosition);
  }

  @Override
  public void periodic() {
    var cameraOneResult = cameraOne.getLatestResult();
    var cameraTwoResult = cameraTwo.getLatestResult();
    var cameraThreeResult = cameraThree.getLatestResult();
    //var cameraFourResult = cameraFour.getLatestResult();

    //var fuelResult = fuelCameraOne.getLatestResult();

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

    /* if (cameraFourResult.hasTargets()) {
      SmartDashboard.putBoolean("fourHasTarget", true);
    } else {
      SmartDashboard.putBoolean("fourHasResult", false);
    } 

    if (fuelResult.hasTargets()) {
      SmartDashboard.putBoolean("fuelHasTarget", true);
      SmartDashboard.putNumber("bestFuelPositionX", fuelResult.getBestTarget().bestCameraToTarget.getX());
      SmartDashboard.putNumber("bestFuelPositionY", fuelResult.getBestTarget().bestCameraToTarget.getY());
      SmartDashboard.putNumber("bestFuelPositionZ", fuelResult.getBestTarget().bestCameraToTarget.getZ());
    } else {
      SmartDashboard.putBoolean("fuelHasTarget", false);
    } */

    Optional<EstimatedRobotPose> cameraOnePose = cameraOnePoseEstimator.estimateCoprocMultiTagPose(cameraOneResult);
    Optional<EstimatedRobotPose> cameraTwoPose = cameraTwoPoseEstimator.estimateCoprocMultiTagPose(cameraTwoResult);
    Optional<EstimatedRobotPose> cameraThreePose = cameraThreePoseEstimator.estimateCoprocMultiTagPose(cameraThreeResult);
    //Optional<EstimatedRobotPose> cameraFourPose = cameraFourPoseEstimator.estimateCoprocMultiTagPose(cameraFourResult);

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

    /* 
    if (cameraFourPose.isPresent()) {
      SmartDashboard.putBoolean("camFourPresent", true);
      swerveDrivetrain.addVisionMeasurement(
        cameraFourPose.get().estimatedPose.toPose2d(), 
        cameraFourPose.get().timestampSeconds);
    } else {
      SmartDashboard.putBoolean("camFourPresent", false);
    } */
  }
}
