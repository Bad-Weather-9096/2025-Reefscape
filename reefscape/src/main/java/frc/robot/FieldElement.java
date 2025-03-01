package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class FieldElement {
    public enum Type {
        CORAL_STATION(55.25, 8.0),
        PROCESSOR(47.88, 8.0),
        BARGE(70.73, 0.0),
        REEF(8.75, 0.0);

        private final Distance height;
        private final Distance standoff;

        Type(double height, double standoff) {
            this.height = Units.Inches.of(height);
            this.standoff = Units.Inches.of(standoff);
        }

        public Distance getHeight() {
            return height;
        }

        public Distance getStandoff() {
            return standoff;
        }
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
