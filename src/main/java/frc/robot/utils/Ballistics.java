package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Ballistics {
    /* TODO:
     * Implement math for actually calculating velocity + angle
     */

    private final CommandSwerveDrivetrain drivetrain;

    public Ballistics(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    public double calculateShooterAngle() {
        /* This is just a placeholder value for testing. */
        return -2.023;
    }

    public double calculateInitialShooterRPM() {
        /* This is just a placeholder value for testing. */

        /* 6000 RPM absolute max, 5500 max for good recovery */

        //rpm = 14.26*distance+1467

        //distance in inches

        double rpm = Math.floor((14.26*(Units.metersToInches(drivetrain.getDistanceFromHub())+0.4064)) + 1467);

        return -rpm;
    }

    public double calculateInitialHandoffRPM() {

        return -(calculateInitialShooterRPM())-100 ;
    }
}
