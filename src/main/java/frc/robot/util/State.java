package frc.robot.util;

public class State {
    public enum RobotState {
        IDLE,
        AIMING
    }

    public static RobotState currentState = RobotState.IDLE;
}
