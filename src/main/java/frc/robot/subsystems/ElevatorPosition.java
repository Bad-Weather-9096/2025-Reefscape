package frc.robot.subsystems;

public enum ElevatorPosition {
    BASE(0.0),
    RELEASE_LOWER_CORAL(99.0),
    RELEASE_UPPER_CORAL(154.0);

    private final double rotations;

    ElevatorPosition(double rotations) {
        this.rotations = rotations;
    }

    public double getRotations() {
        return rotations;
    }
}
