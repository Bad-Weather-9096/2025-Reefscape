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
    private enum Operation {
        SHIFT,
        EXTRACT_ALGAE
    }

    private XboxController driveController = new XboxController(0);
    private XboxController elevatorController = new XboxController(1);

    private DriveSubsystem driveSubsystem = new DriveSubsystem();
    private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private double tx = 0.0;

    private int fiducialID = -1;

    private Operation operation = null;

    private long end = Long.MIN_VALUE;

    private static final String LIMELIGHT_URL = "http://10.90.96.11:5800";

    private static final double REVERSE_DISTANCE = 36.0; // inches
    private static final double REVERSE_TIME = 4.0; // seconds

    private static final double DRIVE_DEADBAND = 0.05;
    private static final double INTAKE_DEADBAND = 0.02;

    private static final double CORAL_STATION_OFFSET = 8.0; // inches
    private static final double REEF_OFFSET = 6.75; // inches

    private static final double SHIFT_SPEED = 0.25; // percent

    private static final double ALGAE_EXTRACTION_DISTANCE = 12.0; // inches
    private static final double ALGAE_EXTRACTION_TIME = 4.0; // seconds

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
        var tv = LimelightHelpers.getTV(null);

        SmartDashboard.putBoolean("tv", tv);

        if (tv) {
            tx = LimelightHelpers.getTX(null);
            fiducialID = (int)LimelightHelpers.getFiducialID(null);
        }

        SmartDashboard.putNumber("tx", tx);
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
                stop();

                operation = null;
            }
        } else {
            var target = getTarget();

            if (driveController.getLeftBumperButtonPressed() && target != null) {
                switch (target.getType()) {
                    case CORAL_STATION -> shift(-CORAL_STATION_OFFSET);
                    case REEF -> shift(-REEF_OFFSET);
                }
            } else if (driveController.getRightBumperButtonPressed() && target != null) {
                switch (target.getType()) {
                    case CORAL_STATION -> shift(CORAL_STATION_OFFSET);
                    case REEF -> shift(REEF_OFFSET);
                }
            } else {
                var xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
                var ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

                var rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

                boolean fieldRelative;
                if (driveController.getAButton()) {
                    xSpeed *= 0.5;

                    if (target == null) {
                        ySpeed *= 0.5;

                        rot = 0.0;
                    } else {
                        var offset = normalizeAngle(target.getAngle().in(Units.Degrees)) - normalizeAngle(driveSubsystem.getHeading());

                        rot = -offset / 15.0;

                        ySpeed *= Math.max(1.0 - Math.abs(rot), 0.0) * (Math.abs(tx) / 30.0);

                        switch (target.getType()) {
                            case CORAL_STATION -> elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RECEIVE_CORAL);
                            case PROCESSOR -> elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RELEASE_ALGAE);
                        }
                    }

                    fieldRelative = false;
                } else {
                    tx = 0.0;

                    fiducialID = -1;

                    fieldRelative = true;
                }

                driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
            }
        }
    }

    private void operate() {
        elevatorSubsystem.setElevatorSpeed(elevatorController.getLeftY());
        elevatorSubsystem.setEndEffectorPosition(elevatorController.getRightY());

        var leftTrigger = MathUtil.applyDeadband(elevatorController.getLeftTriggerAxis(), INTAKE_DEADBAND);
        var rightTrigger = MathUtil.applyDeadband(elevatorController.getRightTriggerAxis(), INTAKE_DEADBAND);

        elevatorSubsystem.setIntakeSpeed(leftTrigger - rightTrigger);

        if (elevatorController.getAButtonPressed()) {
            elevatorSubsystem.setPosition(ElevatorSubsystem.Position.TARGET_LOWER_TAGS);
        } else if (elevatorController.getBButtonPressed()) {
            elevatorSubsystem.setPosition(ElevatorSubsystem.Position.TARGET_UPPER_TAGS);
        } else if (elevatorController.getXButtonPressed()) {
            if (extractAlgae()) {
                elevatorSubsystem.extractAlgae(ALGAE_EXTRACTION_TIME);
            }
        } else if (elevatorController.getYButtonPressed()) {
            elevatorSubsystem.setPosition(ElevatorSubsystem.Position.TRANSPORT);
        } else if (elevatorController.getLeftBumperButtonPressed()) {
            elevatorSubsystem.receiveCoral();
        } else if (elevatorController.getRightBumperButtonPressed()) {
            elevatorSubsystem.releaseCoral();
        } else {
            var target = getTarget();

            if (target != null) {
                var pov = elevatorController.getPOV();

                if (pov != -1) {
                    switch (pov) {
                        case 0 -> {
                            if (elevatorSubsystem.hasCoral()) {
                                elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RELEASE_UPPER_CORAL);
                            } else {
                                elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RECEIVE_UPPER_ALGAE);
                            }
                        }
                        case 180 -> {
                            if (elevatorSubsystem.hasCoral()) {
                                elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RELEASE_LOWER_CORAL);
                            } else {
                                elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RECEIVE_LOWER_ALGAE);
                            }
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

    private void shift(double distance) {
        if (operation == Operation.SHIFT) {
            return;
        }

        operation = Operation.SHIFT;

        driveSubsystem.drive(0.0, -Math.signum(distance) * SHIFT_SPEED, 0.0, false);

        var dy = Units.Inches.of(distance).in(Units.Meters);

        var t = Math.abs(dy) / (SHIFT_SPEED * Constants.DriveConstants.kMaxSpeedMetersPerSecond);

        end = System.currentTimeMillis() + (long)(t * 1000);
    }

    private boolean extractAlgae() {
        if (operation == Operation.EXTRACT_ALGAE) {
            return false;
        }

        operation = Operation.EXTRACT_ALGAE;

        var vx = Units.InchesPerSecond.of(ALGAE_EXTRACTION_DISTANCE / ALGAE_EXTRACTION_TIME).in(Units.MetersPerSecond);

        var xSpeed = -vx / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        driveSubsystem.drive(xSpeed, 0.0, 0.0, false);

        end = System.currentTimeMillis() + (long)(ALGAE_EXTRACTION_TIME * 1000);

        return true;
    }

    private void stop() {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }
}
