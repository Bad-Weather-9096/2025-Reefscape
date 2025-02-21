package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class FieldElement {
    public enum Type {
        CORAL_STATION(55.25),
        PROCESSOR(47.88),
        BARGE(70.73),
        REEF(8.75);

        private final Distance height;

        Type(double height) {
            this.height = Distance.ofBaseUnits(height, Units.Inches);
        }

        public Distance getHeight() {
            return height;
        }
    }

    private Type type;
    private Angle angle;

    public FieldElement(Type type, double angle) {
        this.type = type;
        this.angle = Angle.ofBaseUnits(angle, Units.Degrees);
    }

    public Type getType() {
        return type;
    }

    public Angle getAngle() {
        return angle;
    }
}
