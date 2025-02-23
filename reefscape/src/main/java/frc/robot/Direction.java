package frc.robot;

public enum Direction {
    UP,
    RIGHT,
    DOWN,
    LEFT;

    private static final Direction[] values = values();

    public static Direction fromAngle(int angle) {
        return values[(angle + 45) % 360];
    }
}
