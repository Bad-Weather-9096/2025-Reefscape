package frc.robot;

import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class UnitsTest {
    @Test
    public void testInchesToMeters() {
        var inches = Units.Inches.of(10);
        var meters = inches.in(Units.Meters);

        Assertions.assertEquals(0.254, meters);
    }
}
