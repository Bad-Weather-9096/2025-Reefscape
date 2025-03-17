package frc.robot.subsystems;

public enum ElevatorPosition {
    BASE(0.0),
    RECEIVE_CORAL(100.0), // TODO
    RELEASE_LOWER_CORAL(125.0), // TODO
    RELEASE_UPPER_CORAL(175.0); // TODO

    private final double rotations;

    ElevatorPosition(double rotations) {
        this.rotations = rotations;
    }

    public double getRotations() {
        return rotations;
    }
}
