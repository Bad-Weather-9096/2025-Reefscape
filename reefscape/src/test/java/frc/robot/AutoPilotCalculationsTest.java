package frc.robot;

import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

public class AutoPilotCalculationsTest {
    private static final double TAG_HEIGHT = 6.5; // inches
    private static final double BASE_HEIGHT = 5.0; // inches
    private static final double CAMERA_HEIGHT = 15.5; // inches
    private static final double CAMERA_INSET = 2.5; // inches

    @Test
    public void testCalculations() {
        logResult("parallel with field element/aligned with tag",
            new FieldElement(FieldElement.Type.REEF, 0.0), 0.0, 0.0, -15.0);

        logResult("parallel with field element/to the left of tag",
            new FieldElement(FieldElement.Type.REEF, 0.0), 0.0, 26.6, -15.0);

        logResult("parallel with field element/to the right of tag",
            new FieldElement(FieldElement.Type.REEF, 0.0), 0.0, -26.6, -15.0);

        logResult("turned 45 degrees CW/to the left of tag",
            new FieldElement(FieldElement.Type.REEF, 0.0), 45.0, -18.4, -15.0);

        logResult("turned 45 degrees CCW/to the right of tag",
            new FieldElement(FieldElement.Type.REEF, 0.0), -45.0, 18.4, -15.0);

        logResult("parallel with field element/aligned with tag",
            new FieldElement(FieldElement.Type.CORAL_STATION, 0.0), 0.0, 0.0, 15.0);

        logResult("parallel with field element/to the left of tag",
            new FieldElement(FieldElement.Type.CORAL_STATION, 0.0), 0.0, 26.6, 15.0);

        logResult("parallel with field element/to the right of tag",
            new FieldElement(FieldElement.Type.CORAL_STATION, 0.0), 0.0, -26.6, 15.0);

        logResult("turned 45 degrees CW/to the left of tag",
            new FieldElement(FieldElement.Type.CORAL_STATION, 0.0), 45.0, -18.4, 15.0);

        logResult("turned 45 degrees CCW/to the right of tag",
            new FieldElement(FieldElement.Type.CORAL_STATION, 0.0), -45.0, 18.4, 15.0);
    }

    private void logResult(String label, FieldElement fieldElement, double yaw, double tx, double ty) {
        System.out.println(label);

        var type = fieldElement.getType();
        var angle = fieldElement.getAngle().in(Units.Radians);

        System.out.printf("type = %s, angle = %.2f\n", type, angle);
        System.out.printf("yaw = %.2f, tx = %.2f, ty = %.2f\n", yaw, tx, ty);

        var ht = type.getHeight().in(Units.Meters) + Units.Inches.of(TAG_HEIGHT).in(Units.Meters) / 2;
        var hc = Units.Inches.of(BASE_HEIGHT + CAMERA_HEIGHT).in(Units.Meters);

        var ci = Units.Inches.of(CAMERA_INSET).in(Units.Meters);

        var heading = Math.toRadians(yaw);

        System.out.printf("ht = %.2f, hc = %.2f, ci = %.2f, heading = %.2f\n", ht, hc, ci, heading);

        var dx = (ht - hc) / Math.tan(Math.toRadians(ty)) - ci;
        var dy = dx * Math.tan(Math.toRadians(tx) + heading);

        var st = type.getStandoff().in(Units.Meters);

        System.out.printf("st = %.2f\n", st);

        dx -= st;

        var a = angle - heading;

        var t = getTime(dx, dy, a);

        var xSpeed = dx / t;
        var ySpeed = dy / t;

        var rot = a / t;

        System.out.printf("dx = %.2f, dy = %.2f, a = %.2f\n", dx, dy, a);
        System.out.printf("xSpeed = %.2f, ySpeed = %.2f, rot = %.2f, t = %.2f\n", xSpeed, ySpeed, rot, t);

        System.out.println();
    }

    private double getTime(double dx, double dy, double a) {
        var tx = Math.abs(dx) / Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        var ty = Math.abs(dy) / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        var ta = Math.abs(a) / Constants.DriveConstants.kMaxAngularSpeed;

        return Math.max(Math.max(tx, ty), ta) * 2.0;
    }
}
