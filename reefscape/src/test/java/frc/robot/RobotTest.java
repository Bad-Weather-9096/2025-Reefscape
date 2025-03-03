package frc.robot;

import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

public class RobotTest {
    private static final double CORAL_STATION_CAMERA_HEIGHT = 40.5; // inches
    private static final double REEF_CAMERA_HEIGHT = 15.5; // inches

    private static final double CAMERA_OFFSET = 13.5; // inches

    private static FieldElement getTarget(int id) {
        return Robot.fieldElements.get(id - 1);
    }

    @Test
    public void testAlignmentParameters() {
        logAlignmentParameters(getTarget(1), -135.0, 10.0, 7.5);
        logAlignmentParameters(getTarget(2), 135.0, -10.0, 7.5);

        logAlignmentParameters(getTarget(7), 30.0, -10.5, -8.2);
        logAlignmentParameters(getTarget(7), -30.0, 10.5, -8.2);

        logAlignmentParameters(getTarget(1), -180.0, 5.5, 12.5);
        logAlignmentParameters(getTarget(1), 180.0, 5.5, 12.5);
    }

    private void logAlignmentParameters(FieldElement target, double heading, double tx, double ty) {
        var cameraHeight = switch (target.getType()) {
            case CORAL_STATION -> CORAL_STATION_CAMERA_HEIGHT;
            case REEF -> REEF_CAMERA_HEIGHT;
            default -> throw new UnsupportedOperationException();
        };

        logAlignmentParameters(target, cameraHeight, heading, tx, ty);
    }

    private void logAlignmentParameters(FieldElement target, double cameraHeight, double heading, double tx, double ty) {
        var alignmentParameters = Robot.getAlignmentParameters(target, cameraHeight, CAMERA_OFFSET, heading, tx, ty);

        System.out.printf("angle = %.2f degrees, heading = %.2f degrees, tx = %.2f degrees\n",
            Robot.normalizeAngle(target.getAngle().in(Units.Degrees)),
            Robot.normalizeAngle(heading),
            tx);

        var a = Math.toDegrees(alignmentParameters.a());
        var dy = Units.Meters.of(alignmentParameters.dy()).in(Units.Inches);

        System.out.printf("a = %.2f degrees, dy = %.2f inches\n", a, dy);
    }
}
