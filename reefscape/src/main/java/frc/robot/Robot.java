package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
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

    private boolean tv = false;

    private double tx = 0.0;
    private double ty = 0.0;

    private int fiducialID = -1;

    private AutonomousMode autonomousMode = null;

    private FieldElement target = null;

    private boolean shifting = false;

    private long end = Integer.MIN_VALUE;

    private static final String LIMELIGHT_URL = "http://10.90.96.11:5800";
    private static final String LIMELIGHT_NAME = "";

    private static final double REVERSE_DISTANCE = 72.0; // inches
    private static final double REVERSE_TIME = 2.0; // seconds

    private static final double LOCATE_TAG_SPEED = Math.PI / 2; // radians/second

    private static final double DRIVE_DEADBAND = 0.05;

    private static final double ELEVATOR_DEADBAND = 0.02;
    private static final double END_EFFECTOR_DEADBAND = 0.02;

    private static final double ALGAE_DEADBAND = 0.05;

    private static final double BASE_HEIGHT = 5.0; // inches
    private static final double CAMERA_INSET = 0.0; // inches

    private static final double CORAL_STATION_OFFSET = 8.0; // inches
    private static final double REEF_OFFSET = 6.5; // inches
    private static final double MOVE_TIME = 1.5; // seconds

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
        tv = LimelightHelpers.getTV(LIMELIGHT_NAME);

        tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        ty = LimelightHelpers.getTY(LIMELIGHT_NAME);

        fiducialID = (int)LimelightHelpers.getFiducialID(LIMELIGHT_NAME);

        SmartDashboard.putBoolean("tv", tv);

        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);

        SmartDashboard.putNumber("fiducial-id", fiducialID);
    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture(new HttpCamera("limelight", LIMELIGHT_URL, HttpCameraKind.kMJPGStreamer));
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putBoolean("target-locked", target != null);

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

            var dr = Distance.ofBaseUnits(REVERSE_DISTANCE, Units.Inches).in(Units.Meters);

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
                    if (tv) {
                        var fieldElement = fieldElements.get(fiducialID - 1);

                        if (fieldElement.getType() == FieldElement.Type.REEF) {
                            autonomousMode = AutonomousMode.DOCK;

                            dock();

                            if (fiducialID == 7
                                || fiducialID == 9
                                || fiducialID == 11
                                || fiducialID == 18
                                || fiducialID == 20
                                || fiducialID == 22) {
                                elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.UPPER_ALGAE);
                            } else {
                                elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.LOWER_ALGAE);
                            }
                        }
                    }
                }
                case DOCK -> {
                    if (now >= end) {
                        autonomousMode = AutonomousMode.DONE;

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

        if (shifting) {
            if (now >= end) {
                stop();

                shifting = false;
            }
        } else if (auxilliaryController.getAButton() && (target != null || tv)) {
            if (target == null) {
                dock();

                switch (target.getType()) {
                    case CORAL_STATION -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.CORAL_INTAKE);
                    case PROCESSOR -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.PROCESSOR);
                    case REEF -> elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.BASE);
                }

                return;
            }

            if (now >= end) {
                stop();
            }
        } else {
            target = null;

            var xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
            var ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

            var rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

            driveSubsystem.drive(xSpeed, ySpeed, rot, true);
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

        if (auxilliaryController.getLeftBumperButtonPressed()) {
            elevatorSubsystem.receiveCoral();
            elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.BASE);
        }

        if (auxilliaryController.getRightBumperButtonPressed()) {
            elevatorSubsystem.releaseCoral();
        }

        if (MathUtil.applyDeadband(auxilliaryController.getLeftTriggerAxis(), ALGAE_DEADBAND) > 0.0) {
            elevatorSubsystem.receiveAlgae();
            elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.PROCESSOR);
        }

        if (MathUtil.applyDeadband(auxilliaryController.getRightTriggerAxis(), ALGAE_DEADBAND) > 0.0) {
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
                                elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.UPPER_CORAL);
                            } else {
                                elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.UPPER_ALGAE);
                            }
                        }
                        case DOWN -> {
                            if (elevatorSubsystem.hasCoral()) {
                                elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.LOWER_CORAL);
                            } else {
                                elevatorSubsystem.adjustPosition(ElevatorSubsystem.Position.LOWER_ALGAE);
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
        var fieldElement = fieldElements.get(fiducialID - 1);

        var ht = fieldElement.getType().getHeight().in(Units.Meters);
        var hc = Distance.ofBaseUnits(BASE_HEIGHT + elevatorSubsystem.getCameraHeight(), Units.Inches).in(Units.Meters);

        var ci = Distance.ofBaseUnits(CAMERA_INSET, Units.Inches).in(Units.Meters);

        var dx = (ht - hc) / Math.tan(Math.toRadians(ty)) - ci;
        var dy = dx * Math.tan(Math.toRadians(tx));

        var angle = fieldElement.getAngle().in(Units.Radians);
        var heading = Math.toRadians(driveSubsystem.getHeading());

        var a = angle - heading;

        var t = getMaximumDockingTime(dx, dy, a);

        var xSpeed = dx / t;
        var ySpeed = dy / t;
        var rot = a / t;

        driveSubsystem.drive(xSpeed, ySpeed, rot, false);

        target = fieldElement;

        end = System.currentTimeMillis() + (long)(t * 1000);
    }

    private double getMaximumDockingTime(double dx, double dy, double a) {
        var tx = dx / Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        var ty = dy / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        var ta = Math.abs(a) / Constants.DriveConstants.kMaxAngularSpeed;

        return Math.max(Math.max(tx, ty), ta) * 2.0;
    }

    private void shift(double distance) {
        if (shifting) {
            return;
        }

        var ySpeed = distance / MOVE_TIME;

        driveSubsystem.drive(0.0, ySpeed, 0.0, false);

        shifting = true;

        end = System.currentTimeMillis() + (long)(MOVE_TIME * 1000);
    }

    private void stop() {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }
}
