package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.List;

/**
 * Robot class.
 */
public class Robot extends TimedRobot {
    public record AlignmentParameters(
        double a, // radians
        double dy // meters
    ) {
    }

    private enum Operation {
        ROTATE,
        TRANSLATE,
        SHIFT,
        EXTRACT_ALGAE
    }

    private XboxController driveController = new XboxController(0);
    private XboxController elevatorController = new XboxController(1);

    private DriveSubsystem driveSubsystem = new DriveSubsystem();
    private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private boolean tv = false;

    private double tx = 0.0;
    private double ty = 0.0;

    private int fiducialID = -1;

    private AlignmentParameters alignmentParameters = null;

    private Operation operation = null;

    private long end = Long.MIN_VALUE;

    private static final String LIMELIGHT_URL = "http://10.90.96.11:5800";

    private static final double TAG_HEIGHT = 6.5; // inches
    private static final double BASE_HEIGHT = 5.0; // inches
    private static final double CAMERA_OFFSET = 13.5; // inches

    private static final double CAMERA_HFOV = 54.0; // degrees

    private static final double REVERSE_DISTANCE = 72.0; // inches
    private static final double REVERSE_TIME = 4.0; // seconds

    private static final double DRIVE_DEADBAND = 0.05;

    private static final double ELEVATOR_DEADBAND = 0.02;
    private static final double END_EFFECTOR_DEADBAND = 0.02;

    private static final double CORAL_STATION_OFFSET = 8.0; // inches
    private static final double REEF_OFFSET = 6.75; // inches

    private static final double EXTRACT_ALGAE_DISTANCE = 12.0; // inches
    private static final double EXTRACT_ALGAE_TIME = 4.0; // seconds

    public static final List<FieldElement> fieldElements = List.of(
        new FieldElement(FieldElement.Type.CORAL_STATION, -126.0),
        new FieldElement(FieldElement.Type.CORAL_STATION, 126.0),
        new FieldElement(FieldElement.Type.PROCESSOR, 90.0),
        new FieldElement(FieldElement.Type.BARGE, 0.0),
        new FieldElement(FieldElement.Type.BARGE, 0.0),
        new FieldElement(FieldElement.Type.REEF, 60.0),
        new FieldElement(FieldElement.Type.REEF, 0.0),
        new FieldElement(FieldElement.Type.REEF, -60.0),
        new FieldElement(FieldElement.Type.REEF, -120.0),
        new FieldElement(FieldElement.Type.REEF, 180.0),
        new FieldElement(FieldElement.Type.REEF, 120.0),
        new FieldElement(FieldElement.Type.CORAL_STATION, 126.0),
        new FieldElement(FieldElement.Type.CORAL_STATION, -126.0),
        new FieldElement(FieldElement.Type.BARGE, 0.0),
        new FieldElement(FieldElement.Type.BARGE, 0.0),
        new FieldElement(FieldElement.Type.PROCESSOR, 90.0),
        new FieldElement(FieldElement.Type.REEF, -60.0),
        new FieldElement(FieldElement.Type.REEF, 0.0),
        new FieldElement(FieldElement.Type.REEF, 60.0),
        new FieldElement(FieldElement.Type.REEF, 120.0),
        new FieldElement(FieldElement.Type.REEF, 180.0),
        new FieldElement(FieldElement.Type.REEF, -120.0)
    );

    public static AlignmentParameters getAlignmentParameters(FieldElement target,
        double cameraHeight, double cameraOffset,
        double heading, double tx, double ty) {
        var type = target.getType();

        var ht = type.getHeight().in(Units.Meters) + Units.Inches.of(TAG_HEIGHT).in(Units.Meters) / 2;
        var hc = Units.Inches.of(BASE_HEIGHT + cameraHeight).in(Units.Meters);

        var a = Math.toRadians(normalizeAngle(target.getAngle().in(Units.Degrees)) - normalizeAngle(heading));

        var dx = (ht - hc) / Math.tan(Math.toRadians(ty));

        var dy1 = dx * Math.tan(a + Math.toRadians(tx));
        var dy2 = Units.Inches.of(cameraOffset).in(Units.Meters) * Math.sin(a);

        return new AlignmentParameters(a, -(dy1 + dy2));
    }

    public static double normalizeAngle(double angle) {
        return (angle + 180.0) % 360.0 - 180.0;
    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture(new HttpCamera("limelight", LIMELIGHT_URL, HttpCameraKind.kMJPGStreamer));
    }

    @Override
    public void robotPeriodic() {
        driveSubsystem.periodic();
        elevatorSubsystem.periodic();
    }

    private void readLimelight() {
        tv = LimelightHelpers.getTV(null);

        SmartDashboard.putBoolean("tv", tv);

        tx = LimelightHelpers.getTX(null);
        ty = LimelightHelpers.getTY(null);

        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);

        if (tv) {
            fiducialID = (int) LimelightHelpers.getFiducialID(null);
        }

        SmartDashboard.putNumber("fiducial-id", fiducialID);
    }

    private FieldElement getTarget() {
        return (fiducialID > 0) ? fieldElements.get(fiducialID - 1) : null;
    }

    @Override
    public void autonomousInit() {
        var dx = Units.Inches.of(REVERSE_DISTANCE).in(Units.Meters);

        var xSpeed = -(dx / REVERSE_TIME) / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        var rot = (Math.PI / REVERSE_TIME) / Constants.DriveConstants.kMaxAngularSpeed;

        driveSubsystem.drive(xSpeed, 0.0, rot, false);

        end = System.currentTimeMillis() + (long)(REVERSE_TIME * 1000);
    }

    @Override
    public void autonomousPeriodic() {
        var now = System.currentTimeMillis();

        if (now >= end) {
            stop();
        }
    }

    @Override
    public void teleopPeriodic() {
        readLimelight();

        navigate();
        operate();
    }

    private void navigate() {
        if (operation != null) {
            var now = System.currentTimeMillis();

            if (now >= end) {
                if (operation == Operation.ROTATE) {
                    translate();
                } else {
                    stop();

                    operation = null;
                }
            }
        } else if (driveController.getAButtonPressed()) {
            var target = getTarget();

            if (target != null) {
                alignmentParameters = getAlignmentParameters(target,
                    elevatorSubsystem.getCameraHeight(), CAMERA_OFFSET,
                    driveSubsystem.getHeading(), tx, ty);

                SmartDashboard.putNumber("alignment-a", alignmentParameters.a());
                SmartDashboard.putNumber("alignment-dy", alignmentParameters.dy());

                rotate();
            }
        } else {
            var xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
            var ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

            var rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

            boolean fieldRelative;
            if (driveController.getBButton()) {
                if (tv) {
                    rot = -tx / (CAMERA_HFOV / 2);
                } else {
                    rot = 0.0;
                }

                fieldRelative = false;
            } else {
                fieldRelative = true;
            }

            driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
        }
    }

    private void operate() {
        var leftY = -MathUtil.applyDeadband(elevatorController.getLeftY(), ELEVATOR_DEADBAND);

        if (leftY < 0.0) {
            elevatorSubsystem.raiseElevator();
        } else if (leftY > 0.0) {
            elevatorSubsystem.lowerElevator();
        } else {
            elevatorSubsystem.stopElevator();
        }

        var rightY = -MathUtil.applyDeadband(elevatorController.getLeftY(), END_EFFECTOR_DEADBAND);

        if (rightY < 0.0) {
            elevatorSubsystem.raiseEndEffector();
        } else if (rightY > 0.0) {
            elevatorSubsystem.lowerEndEffector();
        } else {
            elevatorSubsystem.stopEndEffector();
        }

        var target = getTarget();

        if (target != null) {
            if (elevatorController.getAButtonPressed()) {
                switch (target.getType()) {
                    case CORAL_STATION -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.CORAL_INTAKE);
                    case PROCESSOR -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.ALGAE_RELEASE);
                    case REEF -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.TRANSPORT);
                }
            } else if (elevatorController.getBButtonPressed()) {
                if (target.getType() == FieldElement.Type.REEF) {
                    extractAlgae();
                }
            } else if (elevatorController.getLeftBumperButtonPressed()) {
                switch (target.getType()) {
                    case CORAL_STATION -> shift(CORAL_STATION_OFFSET);
                    case REEF -> shift(REEF_OFFSET);
                }
            } else if (elevatorController.getRightBumperButtonPressed()) {
                switch (target.getType()) {
                    case CORAL_STATION -> shift(-CORAL_STATION_OFFSET);
                    case REEF -> shift(-REEF_OFFSET);
                }
            } else {
                // TODO
            }
        }
    }

    @Override
    public void teleopExit() {
        stop();
    }

    private void rotate() {
        operation = Operation.ROTATE;

        var a = alignmentParameters.a();

        var rot = Constants.DriveConstants.kMaxAngularSpeed / 4;

        driveSubsystem.drive(0.0, 0.0, -Math.signum(a) * rot, false);

        var t = Math.abs(a) / rot;

        end = System.currentTimeMillis() + (long)(t * 1000);
    }

    private void translate() {
        operation = Operation.TRANSLATE;

        var dy = alignmentParameters.dy();

        var ySpeed = Constants.DriveConstants.kMaxSpeedMetersPerSecond / 4;

        driveSubsystem.drive(0.0, Math.signum(dy) * ySpeed, 0.0, false);

        var t = Math.abs(dy) / ySpeed;

        end = System.currentTimeMillis() + (long)(t * 1000);
    }

    private void shift(double distance) {
        if (operation == Operation.SHIFT) {
            return;
        }

        operation = Operation.SHIFT;

        var ySpeed = Constants.DriveConstants.kMaxSpeedMetersPerSecond / 4;

        driveSubsystem.drive(0.0, Math.signum(distance) * ySpeed, 0.0, false);

        var dy = Units.Inches.of(distance).in(Units.Meters);

        var t = Math.abs(dy) / ySpeed;

        end = System.currentTimeMillis() + (long)(t * 1000);
    }

    private void extractAlgae() {
        if (operation == Operation.EXTRACT_ALGAE) {
            return;
        }

        operation = Operation.EXTRACT_ALGAE;

        var dx = Units.Inches.of(EXTRACT_ALGAE_DISTANCE).in(Units.Meters);

        var xSpeed = -(dx / EXTRACT_ALGAE_TIME) / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        driveSubsystem.drive(xSpeed, 0.0, 0.0, false);

        elevatorSubsystem.extractAlgae();

        end = System.currentTimeMillis() + (long)(EXTRACT_ALGAE_TIME * 1000);
    }

    private void stop() {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }
}
