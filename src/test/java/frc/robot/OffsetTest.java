package frc.robot;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class OffsetTest {
    @Test
    public void testGetOffset() {
        assertEquals(0.0, Robot.getOffset(0.0, 0.0));
        assertEquals(-15.0, Robot.getOffset(0.0, 15.0));
        assertEquals(15.0, Robot.getOffset(0.0, -15.0));

        assertEquals(0.0, Robot.getOffset(60.0, 60.0));
        assertEquals(-15.0, Robot.getOffset(60.0, 75.0));
        assertEquals(15.0, Robot.getOffset(60.0, 45.0));

        assertEquals(0.0, Robot.getOffset(120.0, 120.0));
        assertEquals(-15.0, Robot.getOffset(120.0, 135.0));
        assertEquals(15.0, Robot.getOffset(120.0, 105.0));

        assertEquals(0.0, Robot.getOffset(180.0, 180.0));
        assertEquals(-15.0, Robot.getOffset(180.0, -165.0));
        assertEquals(15.0, Robot.getOffset(180.0, 165.0));

        assertEquals(0.0, Robot.getOffset(-120.0, -120.0));
        assertEquals(-15.0, Robot.getOffset(-120.0, -105.0));
        assertEquals(15.0, Robot.getOffset(-120.0, -135.0));

        assertEquals(0.0, Robot.getOffset(-60.0, -60.0));
        assertEquals(-15.0, Robot.getOffset(-60.0, -45.0));
        assertEquals(15.0, Robot.getOffset(-60.0, -75.0));
    }
}
