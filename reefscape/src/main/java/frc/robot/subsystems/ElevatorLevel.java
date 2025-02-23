package frc.robot.subsystems;

public enum ElevatorLevel {
    // TODO
    BASE(0.0),
    CORAL_INTAKE(0.0),
    UPPER_ALGAE(0.0),
    LOWER_ALGAE(0.0),
    UPPER_CORAL_BRANCH(0.0),
    LOWER_CORAL_BRANCH(0.0);

    private final double height; // inches

    ElevatorLevel(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
