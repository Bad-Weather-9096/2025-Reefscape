package frc.robot;

import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

public class NavigationTest {
    private static final FieldElement reef = new FieldElement(FieldElement.Type.REEF, 0.0);
    private static final FieldElement coralStation = new FieldElement(FieldElement.Type.CORAL_STATION, 135.0);

    private static final double CAMERA_HEIGHT_LOW = 16.0; // inches
    private static final double CAMERA_HEIGHT_HIGH = 42.0; // inches

    @Test
    public void testReef() {
        logResult("parallel with field element/aligned with tag", reef, 0.0, -5.0, 0.0);
        logResult("parallel with field element/to the left of tag", reef, 30.0, -5.0, 0.0);
        logResult("parallel with field element/to the right of tag", reef, -30.0, -5.0, 0.0);
        logResult("turned 45 degrees CW/to the left of tag", reef, -15.0, -5.0, 45.0);
        logResult("turned 45 degrees CCW/to the right of tag", reef, 15.0, -5.0, -45.0);
    }

    @Test
    public void testCoralStation() {
        logResult("parallel with field element/aligned with tag", coralStation, 0.0, 7.5, 135.0);
        logResult("parallel with field element/to the left of tag", coralStation, 30.0, 7.5, 135.0);
        logResult("parallel with field element/to the right of tag", coralStation, -30.0, 7.5, 135.0);
        logResult("turned 45 degrees CW/to the left of tag", coralStation, -15.0, 7.5, 180.0);
        logResult("turned 45 degrees CCW/to the right of tag", coralStation, 15.0, 7.5, 90.0);
    }

    private void logResult(String label, FieldElement target, double tx, double ty, double heading) {
        System.out.println(label);

        var type = target.getType();
        var angle = target.getAngle().in(Units.Degrees);

        System.out.printf("type = %s, angle = %.2f\n", type, angle);
        System.out.printf("tx = %.2f, ty = %.2f, heading = %.2f\n", tx, ty, heading);

        var cameraHeight = (type == FieldElement.Type.REEF) ? CAMERA_HEIGHT_LOW : CAMERA_HEIGHT_HIGH;

        var location = Robot.getLocation(target, cameraHeight, tx, ty, heading);

        var dx = location.getX();
        var dy = location.getY();

        var a = Math.toRadians(angle - heading);

        var t = getTime(dx, dy, a);

        System.out.printf("dx = %.2f, dy = %.2f, a = %.2f\n", dx, dy, a);

        var xSpeed = dx / t;
        var ySpeed = dy / t;

        var rot = a / t;

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
