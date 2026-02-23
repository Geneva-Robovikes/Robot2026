package frc.robot.utils;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Pathfind {
    private PathPlannerPath path;
    private PathConstraints constraints;

    private final CommandSwerveDrivetrain drivetrain;

    public Pathfind(CommandSwerveDrivetrain drivetrain) {
        constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

        this.drivetrain = drivetrain;
    }

    private void load(String pathToLoad) {
        try {
            path = PathPlannerPath.fromPathFile(pathToLoad);
        } catch (Exception e) {
            System.out.println("exception");
        }
    }

    public Command to(Location location) {
        System.out.println("biutton");
        switch (location) {
            case CLIMB:
                System.out.println("loading");
                load("climb");
                Command pathfindingCommand = drivetrain.getPathfindingCommand(path, constraints).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

                return pathfindingCommand;
            default:
                System.out.println("sometin wong");
                return new Command() {};
        }
    }
}
