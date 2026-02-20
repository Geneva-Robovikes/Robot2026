package frc.robot.utils;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionDemo extends Command {
    public final PhotonCamera cameraThree;

    public final PIDController yPIDController;
    public final PIDController xPIDController;

    public double xSpeed;
    public double ySpeed;

    public VisionDemo() {
        cameraThree = new PhotonCamera("cameraThree");

        yPIDController = new PIDController(1.0, 0.0, 0.0);
        xPIDController = new PIDController(1.0, 0.0, 0.0);

        xSpeed = 0;
        ySpeed = 0;
    }

    public Optional<PhotonTrackedTarget> targetInView() {
        var result = cameraThree.getLatestResult();
        var target = result.getBestTarget();

        if(result.hasTargets()) {
            SmartDashboard.putBoolean("demoTarget", true);
            return Optional.of(target);
        } else {
            SmartDashboard.putBoolean("demoTarget", false);
            return Optional.empty();
        }
    }

    public double getRobotSpeedX() {
        Optional<PhotonTrackedTarget> optionalTarget = targetInView();

        if (optionalTarget.isPresent()) {
            PhotonTrackedTarget target = optionalTarget.get();

            SmartDashboard.putNumber("targetPitch", target.getPitch());
            SmartDashboard.putNumber("targetYaw", target.getYaw());
            SmartDashboard.putNumber("targetSkew", target.getSkew());

            SmartDashboard.putNumber("targetX", target.bestCameraToTarget.getX());
            SmartDashboard.putNumber("targetY", target.bestCameraToTarget.getY());
            SmartDashboard.putNumber("targetZ", target.bestCameraToTarget.getZ());

            ySpeed = yPIDController.calculate(target.bestCameraToTarget.getY(), 0);
            xSpeed = xPIDController.calculate(target.bestCameraToTarget.getX(), 1);

            SmartDashboard.putNumber("xSpeed", xSpeed * 4);
            SmartDashboard.putNumber("ySpeed", ySpeed * 4);

            return xSpeed * 4;
        } else {
            return 0;
        }
    }

    public double getRobotSpeedY() {
        return ySpeed * 4;
    }
}
