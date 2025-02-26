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
    private XboxController driveController = new XboxController(0);
    private XboxController auxilliaryController = new XboxController(1);

    private DriveSubsystem driveSubsystem = new DriveSubsystem();
    private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private double tx = 0.0;
    private double ty = 0.0;

    private int fiducialID = -1;

    private AutonomousMode autonomousMode = null;

    private boolean docking = false;
    private boolean shifting = false;
    private boolean extractingAlgae = false;

    private long end = Long.MIN_VALUE;

    private static final String LIMELIGHT_URL = "http://10.90.96.11:5800";
    private static final String LIMELIGHT_NAME = "";

    private static final double REVERSE_DISTANCE = 72.0; // inches
    private static final double REVERSE_TIME = 4.0; // seconds

    private static final double LOCATE_TAG_SPEED = Math.PI / 2; // radians/second

    private static final double DRIVE_DEADBAND = 0.05;

    private static final double NUDGE_SPEED = 0.1; // percent

    private static final double ELEVATOR_DEADBAND = 0.02;
    private static final double END_EFFECTOR_DEADBAND = 0.02;

    private static final double ALGAE_INTAKE_DEADBAND = 0.05;

    private static final double CORAL_STATION_OFFSET = 8.0; // inches
    private static final double REEF_OFFSET = 6.5; // inches
    private static final double SHIFT_TIME = 1.5; // seconds

    private static final double TAG_HEIGHT = 6.5; // inches
    private static final double BASE_HEIGHT = 5.0; // inches
    private static final double CAMERA_INSET = 2.5; // inches

    private static final double ALGAE_EXTRACTION_SPEED = 0.05; // percent
    private static final double ALGAE_EXTRACTION_TIME = 4.0; // seconds

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

    private void readLimelight() {
        tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        ty = LimelightHelpers.getTY(LIMELIGHT_NAME);

        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);

        var tv = LimelightHelpers.getTV(LIMELIGHT_NAME);

        SmartDashboard.putBoolean("tv", tv);

        if (tv) {
            fiducialID = (int)LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
        }

        SmartDashboard.putNumber("fiducial-id", fiducialID);
    }

    private FieldElement getTarget() {
        return (fiducialID == -1) ? null : fieldElements.get(fiducialID - 1);
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

    @Override
    public void autonomousInit() {
        autonomousMode = null;
    }

    @Override
    public void autonomousPeriodic() {
        readLimelight();

        var now = System.currentTimeMillis();

        if (autonomousMode == null) {
            autonomousMode = AutonomousMode.REVERSE;

            var dr = Units.Inches.of(REVERSE_DISTANCE).in(Units.Meters);

            var xSpeed = -(dr / REVERSE_TIME) / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

            driveSubsystem.drive(xSpeed, 0.0, 0.0, false);

            end = now + (long)(REVERSE_TIME * 1000);
        } else {
            switch (autonomousMode) {
                case REVERSE -> {
                    if (now >= end) {
                        autonomousMode = AutonomousMode.LOCATE_TAG;

                        var rot = LOCATE_TAG_SPEED / Constants.DriveConstants.kMaxAngularSpeed;

                        driveSubsystem.drive(0.0, 0.0, rot, false);
                    }
                }
                case LOCATE_TAG -> {
                    var target = getTarget();

                    if (target != null && target.getType() == FieldElement.Type.REEF) {
                        autonomousMode = AutonomousMode.DOCK;

                        dock();

                        if (fiducialID == 7
                            || fiducialID == 9
                            || fiducialID == 11
                            || fiducialID == 18
                            || fiducialID == 20
                            || fiducialID == 22) {
                            elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.UPPER_ALGAE_INTAKE);
                        } else {
                            elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.LOWER_ALGAE_INTAKE);
                        }
                    }
                }
                case DOCK -> {
                    if (now >= end) {
                        autonomousMode = AutonomousMode.DONE;

                        docking = false;

                        stop();
                    }
                }
                case DONE -> {
                    // No-op
                }
            }
        }
    }

    @Override
    public void autonomousExit() {
        stop();
    }

    @Override
    public void teleopPeriodic() {
        readLimelight();

        navigate();
        operate();
    }

    private void navigate() {
        var now = System.currentTimeMillis();

        if (driveController.getXButtonPressed()) {
            docking = false;
        }

        if (docking) {
            if (now >= end) {
                stop();

                docking = false;
            }
        } else if (shifting) {
            if (now >= end) {
                stop();

                shifting = false;
            }
        } else if (extractingAlgae) {
            if (now >= end) {
                stop();

                extractingAlgae = false;
            }
        } else {
            var target = getTarget();

            // TODO This could re-initiate docking after completion
            // TODO It could also dock to the previous tag; only set target if A or B is pressed in readLimelight()?
            if (auxilliaryController.getAButton() && target != null) {
                dock();

                switch (target.getType()) {
                    case CORAL_STATION -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.CORAL_INTAKE);
                    case PROCESSOR -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.ALGAE_RELEASE);
                    case REEF -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.TRANSPORT);
                }
            } else {
                var pov = auxilliaryController.getPOV();

                if (pov != -1) {
                    var direction = Direction.fromAngle(pov);

                    var xSpeed = switch (direction) {
                        case UP -> NUDGE_SPEED;
                        case DOWN -> -NUDGE_SPEED;
                        default -> 0.0;
                    };

                    var ySpeed = switch (direction) {
                        case LEFT -> -NUDGE_SPEED;
                        case RIGHT -> NUDGE_SPEED;
                        default -> 0.0;
                    };

                    driveSubsystem.drive(xSpeed, ySpeed, 0.0, false);
                } else {
                    var xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
                    var ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

                    var rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

                    if (auxilliaryController.getBButton() && target != null) {
                        var heading = driveSubsystem.getHeading();

                        if ((int)Math.signum(tx + heading) != (int)Math.signum(ySpeed)) {
                            ySpeed = 0.0;
                        }

                        var angle = target.getAngle().baseUnitMagnitude();

                        if ((int)Math.signum(angle - heading) != (int)Math.signum(rot)) {
                            rot = 0.0;
                        }
                    }

                    driveSubsystem.drive(xSpeed, ySpeed, rot, true);
                }
            }
        }
    }

    private void operate() {
        var leftY = -MathUtil.applyDeadband(auxilliaryController.getLeftY(), ELEVATOR_DEADBAND);

        if (leftY < 0.0) {
            elevatorSubsystem.raiseElevator();
        } else if (leftY > 0.0) {
            elevatorSubsystem.lowerElevator();
        } else {
            elevatorSubsystem.stopElevator();
        }

        var rightY = -MathUtil.applyDeadband(auxilliaryController.getLeftY(), END_EFFECTOR_DEADBAND);

        if (rightY < 0.0) {
            elevatorSubsystem.raiseEndEffector();
        } else if (rightY > 0.0) {
            elevatorSubsystem.lowerEndEffector();
        } else {
            elevatorSubsystem.stopEndEffector();
        }

        var target = getTarget();

        if (auxilliaryController.getXButtonPressed() && target != null && target.getType() == FieldElement.Type.REEF) {
            extractAlgae();
        }

        if (auxilliaryController.getLeftBumperButtonPressed()) {
            elevatorSubsystem.receiveCoral();
        }

        if (auxilliaryController.getRightBumperButtonPressed()) {
            elevatorSubsystem.releaseCoral();
        }

        if (MathUtil.applyDeadband(auxilliaryController.getLeftTriggerAxis(), ALGAE_INTAKE_DEADBAND) > 0.0) {
            elevatorSubsystem.receiveAlgae();
        }

        if (MathUtil.applyDeadband(auxilliaryController.getRightTriggerAxis(), ALGAE_INTAKE_DEADBAND) > 0.0) {
            elevatorSubsystem.releaseAlgae();
        }

        var pov = auxilliaryController.getPOV();

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

    @Override
    public void teleopExit() {
        stop();
    }

    private void dock() {
        if (docking) {
            return;
        }

        var target = getTarget();

        var type = target.getType();
        var angle = target.getAngle().in(Units.Radians);

        var ht = type.getHeight().in(Units.Meters) + Units.Inches.of(TAG_HEIGHT).in(Units.Meters) / 2;
        var hc = Units.Inches.of(BASE_HEIGHT + elevatorSubsystem.getCameraHeight()).in(Units.Meters);

        var heading = Math.toRadians(driveSubsystem.getHeading());

        var dx = (ht - hc) / Math.tan(Math.toRadians(ty));
        var dy = dx * Math.tan(Math.toRadians(tx) + heading);

        var st = type.getStandoff().in(Units.Meters);
        var ci = Units.Inches.of(CAMERA_INSET).in(Units.Meters);

        dx -= (st + ci);

        var a = angle - heading;

        var t = getTime(dx, dy, a);

        var xSpeed = dx / t;
        var ySpeed = dy / t;

        var rot = a / t;

        driveSubsystem.drive(xSpeed, ySpeed, rot, false);

        docking = true;

        end = System.currentTimeMillis() + (long)(t * 1000);
    }

    private double getTime(double dx, double dy, double a) {
        var tx = Math.abs(dx) / Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        var ty = Math.abs(dy) / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        var ta = Math.abs(a) / Constants.DriveConstants.kMaxAngularSpeed;

        return Math.max(Math.max(tx, ty), ta) * 2.0;
    }

    private void shift(double distance) {
        if (shifting) {
            return;
        }

        var ySpeed = distance / SHIFT_TIME;

        driveSubsystem.drive(0.0, ySpeed, 0.0, false);

        shifting = true;

        end = System.currentTimeMillis() + (long)(SHIFT_TIME * 1000);
    }

    private void extractAlgae() {
        if (extractingAlgae) {
            return;
        }

        driveSubsystem.drive(ALGAE_EXTRACTION_SPEED, 0.0, 0.0, false);

        elevatorSubsystem.extractAlgae();

        extractingAlgae = true;

        end = System.currentTimeMillis() + (long)(ALGAE_EXTRACTION_TIME * 1000);
    }

    private void stop() {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }
}
