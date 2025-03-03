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
    private static final double CAMERA_OFFSET = -5.5; // inches

    private static final double CAMERA_HFOV = 54.0; // degrees

    private static final double REVERSE_DISTANCE = 72.0; // inches
    private static final double REVERSE_TIME = 4.0; // seconds

    private static final double DRIVE_DEADBAND = 0.05;

    private static final double ELEVATOR_DEADBAND = 0.02;
    private static final double END_EFFECTOR_DEADBAND = 0.02;

    private static final double ALGAE_INTAKE_DEADBAND = 0.05;

    private static final double CORAL_STATION_OFFSET = 8.0; // inches
    private static final double REEF_OFFSET = 6.5; // inches

    private static final double EXTRACT_ALGAE_DISTANCE = 12.0; // inches
    private static final double EXTRACT_ALGAE_TIME = 4.0; // seconds

    private static final List<FieldElement> fieldElements = List.of(
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

        var a = Math.toRadians(heading) - target.getAngle().in(Units.Radians);

        var dx = (ht - hc) / Math.tan(Math.toRadians(ty));
        var dy = dx * Math.tan(a + Math.toRadians(tx)) + Units.Inches.of(cameraOffset).in(Units.Meters) * Math.sin(a);

        return new AlignmentParameters(a, dy);
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

        fiducialID = (int)LimelightHelpers.getFiducialID(null);

        SmartDashboard.putNumber("fiducial-id", fiducialID);
    }

    private FieldElement getTarget() {
        return tv ? fieldElements.get(fiducialID - 1) : null;
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

        if (elevatorController.getAButtonPressed()) {
            if (target != null) {
                switch (target.getType()) {
                    case CORAL_STATION -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.CORAL_INTAKE);
                    case PROCESSOR -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.ALGAE_RELEASE);
                    case REEF -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.TRANSPORT);
                }
            }
        } else if (elevatorController.getBButtonPressed()) {
            if (target.getType() == FieldElement.Type.REEF) {
                extractAlgae();
            }
        } else if (elevatorController.getLeftBumperButtonPressed()) {
            elevatorSubsystem.receiveCoral();
        } else if (elevatorController.getRightBumperButtonPressed()) {
            elevatorSubsystem.releaseCoral();
        } else if (MathUtil.applyDeadband(elevatorController.getLeftTriggerAxis(), ALGAE_INTAKE_DEADBAND) > 0.0) {
            elevatorSubsystem.receiveAlgae();
        } else if (MathUtil.applyDeadband(elevatorController.getRightTriggerAxis(), ALGAE_INTAKE_DEADBAND) > 0.0) {
            elevatorSubsystem.releaseAlgae();
        } else {
            var pov = elevatorController.getPOV();

            if (pov != -1 && target != null) {
                var direction = Direction.fromAngle(pov);

                switch (target.getType()) {
                    case CORAL_STATION -> {
                        switch (direction) {
                            case LEFT -> shift(-CORAL_STATION_OFFSET);
                            case RIGHT -> shift(CORAL_STATION_OFFSET);
                        }
                    }
                    case REEF -> {
                        switch (direction) {
                            case UP -> {
                                if (elevatorSubsystem.hasCoral()) {
                                    elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.UPPER_CORAL_RELEASE);
                                } else {
                                    elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.UPPER_ALGAE_INTAKE);
                                }
                            }
                            case DOWN -> {
                                if (elevatorSubsystem.hasCoral()) {
                                    elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.LOWER_CORAL_RELEASE);
                                } else {
                                    elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.LOWER_ALGAE_INTAKE);
                                }
                            }
                            case LEFT -> shift(-REEF_OFFSET);
                            case RIGHT -> shift(REEF_OFFSET);
                        }
                    }
                }
            }
        }
    }

    @Override
    public void teleopExit() {
        stop();
    }

    private void rotate() {
        operation = Operation.ROTATE;

        var rot = Constants.DriveConstants.kMaxAngularSpeed / 2;

        driveSubsystem.drive(0.0, 0.0, rot, false);

        var t = Math.abs(alignmentParameters.a()) / rot;

        end = System.currentTimeMillis() + (long)(t * 1000);
    }

    private void translate() {
        operation = Operation.TRANSLATE;

        var ySpeed = Constants.DriveConstants.kMaxSpeedMetersPerSecond / 2;

        driveSubsystem.drive(0.0, ySpeed, 0.0, false);

        var t = Math.abs(alignmentParameters.dy()) / ySpeed;

        end = System.currentTimeMillis() + (long)(t * 1000);
    }

    private void shift(double distance) {
        if (operation == Operation.SHIFT) {
            return;
        }

        operation = Operation.SHIFT;

        var ySpeed = Constants.DriveConstants.kMaxSpeedMetersPerSecond / 2;

        driveSubsystem.drive(0.0, ySpeed, 0.0, false);

        var dy = Units.Inches.of(distance).in(Units.Meters);

        var t = dy / ySpeed;

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
