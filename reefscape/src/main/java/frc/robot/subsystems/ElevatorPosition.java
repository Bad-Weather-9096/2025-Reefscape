package frc.robot.subsystems;

public enum ElevatorPosition {
    // TODO
    BASE(0.0, 0.0),
    PROCESSOR(0.0, 0.0),
    CORAL_INTAKE(0.0, 0.0),
    LOWER_ALGAE(0.0, 0.0),
    UPPER_ALGAE(0.0, 0.0),
    LOWER_CORAL(0.0, 0.0),
    UPPER_CORAL(0.0, 0.0);

    private final double height; // inches
    private final double angle; // degrees

    ElevatorPosition(double height, double angle) {
        this.height = height;
        this.angle = angle;
    }

    public double getHeight() {
        return height;
    }

    public double getAngle() {
        return angle;
    }
}
