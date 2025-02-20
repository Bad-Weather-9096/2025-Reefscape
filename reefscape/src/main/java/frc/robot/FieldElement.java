package frc.robot;

public enum FieldElement {
    CORAL_STATION(55.25),
    PROCESSOR(47.88),
    BARGE(70.73),
    REEF(8.75);

    private final double height;

    FieldElement(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }

    public double getHeightInMeters() {
        return getHeight() * 0.0254;
    }
}
