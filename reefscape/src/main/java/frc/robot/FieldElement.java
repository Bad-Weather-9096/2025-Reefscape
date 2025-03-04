package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public final class FieldElement {
    public enum Type {
        CORAL_STATION,
        PROCESSOR,
        BARGE,
        REEF
    }

    private Type type;
    private Angle angle;

    public FieldElement(Type type, double angle) {
        this.type = type;
        this.angle = Units.Degrees.of(angle);
    }

    public Type getType() {
        return type;
    }

    public Angle getAngle() {
        return angle;
    }
}
