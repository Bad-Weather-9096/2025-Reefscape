package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * Represents a field element.
 */
public enum FieldElement {
    CORAL_STATION(Distance.ofBaseUnits(55.25, Units.Inches)),
    PROCESSOR(Distance.ofBaseUnits(47.88, Units.Inches)),
    BARGE(Distance.ofBaseUnits(70.73, Units.Inches)),
    REEF(Distance.ofBaseUnits(8.75, Units.Inches));

    private final Distance height;

    FieldElement(Distance height) {
        this.height = height;
    }

    public Distance getHeight() {
        return height;
    }
}
