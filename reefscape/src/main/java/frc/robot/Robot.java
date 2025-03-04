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
        SHIFT
    }

    private XboxController driveController = new XboxController(0);
    private XboxController elevatorController = new XboxController(1);

    private DriveSubsystem driveSubsystem = new DriveSubsystem();
    private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private boolean tv = false;

    private double tx = 0.0;
    private double ty = 0.0;

    private int fiducialID = -1;

    private Operation operation = null;

    private long end = Long.MIN_VALUE;

    private static final String LIMELIGHT_URL = "http://10.90.96.11:5800";

    private static final double REVERSE_DISTANCE = 72.0; // inches
    private static final double REVERSE_TIME = 4.0; // seconds

    private static final double DRIVE_DEADBAND = 0.05;

    private static final double ELEVATOR_DEADBAND = 0.02;
    private static final double END_EFFECTOR_DEADBAND = 0.02;

    private static final double CORAL_STATION_OFFSET = 8.0; // inches
    private static final double REEF_OFFSET = 6.75; // inches

    private static final double ALIGNMENT_THRESHOLD = 2.5; // degrees
    private static final double ALIGNMENT_SPEED = 0.5; // percent

    private static final double SHIFT_SPEED = 0.25; // percent

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
                stop();

                operation = null;
            }
        } else {
            var target = getTarget();

            if (elevatorController.getLeftBumperButtonPressed() && target != null) {
                switch (target.getType()) {
                    case CORAL_STATION -> shift(-CORAL_STATION_OFFSET);
                    case REEF -> shift(-REEF_OFFSET);
                }
            } else if (elevatorController.getRightBumperButtonPressed() && target != null) {
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
                    if (target != null) {
                        var a = normalizeAngle(target.getAngle().in(Units.Degrees)) - normalizeAngle(driveSubsystem.getHeading());

                        if (Math.abs(a) > ALIGNMENT_THRESHOLD) {
                            ySpeed = 0.0;

                            rot = -Math.signum(a) * ALIGNMENT_SPEED;
                        } else {
                            rot = 0.0;
                        }
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

    private void stop() {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }
}
