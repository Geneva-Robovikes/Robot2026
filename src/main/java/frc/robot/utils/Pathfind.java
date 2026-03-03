package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class Pathfind {
    private PathPlannerPath path;
    private PathConstraints constraints;

    public Pathfind() {
        constraints = new PathConstraints(
        4.0, 5.2,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    }

    private void load(String pathToLoad) {
        try {
            path = PathPlannerPath.fromPathFile(pathToLoad);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public Command to(Location location) {
        switch (location) {
            case DRIVER_RIGHT_CLIMB:
                load("climbr");

                if (AutoBuilder.isConfigured()) {
                    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);

                    return pathfindingCommand;
                }

            case DRIVER_LEFT_CLIMB:
                load("climbl");

                if (AutoBuilder.isConfigured()) {
                    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);

                    return pathfindingCommand;
                }

            default:
                System.out.println("sometin wong");
                return new Command() {};
        }
    }
}
