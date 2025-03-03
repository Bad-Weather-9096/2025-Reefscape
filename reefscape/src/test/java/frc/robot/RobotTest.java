package frc.robot;

import org.junit.jupiter.api.Test;

public class RobotTest {
    private static final double REEF_CAMERA_HEIGHT = 15.5; // inches
    private static final double CENTER_CAMERA_OFFSET = 0.0; // inches

    @Test
    public void testAlignmentParameters() {
        logAlignmentParameters(new FieldElement(FieldElement.Type.REEF, 0.0),
            REEF_CAMERA_HEIGHT, CENTER_CAMERA_OFFSET,
            0.0, 0.0, -7.5);

        logAlignmentParameters(new FieldElement(FieldElement.Type.REEF, 0.0),
            REEF_CAMERA_HEIGHT, CENTER_CAMERA_OFFSET,
            30.0, -15.0, -7.5);

        logAlignmentParameters(new FieldElement(FieldElement.Type.REEF, 0.0),
            REEF_CAMERA_HEIGHT, CENTER_CAMERA_OFFSET,
            -30.0, 15.0, -7.5);
    }

    private void logAlignmentParameters(FieldElement target,
        double cameraHeight, double cameraOffset,
        double heading, double tx, double ty) {
        System.out.println(Robot.getAlignmentParameters(target, cameraHeight, cameraOffset, heading, tx, ty));
    }
}
