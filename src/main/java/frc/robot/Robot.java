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
import frc.robot.subsystems.ElevatorPosition;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.Map;

/**
 * Robot class.
 */
public class Robot extends TimedRobot {
    private XboxController driveController = new XboxController(0);
    private XboxController elevatorController = new XboxController(1);

    private DriveSubsystem driveSubsystem = new DriveSubsystem();
    private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private double tx = 0.0;

    private int fiducialID = -1;

    private boolean shifting = false;

    private long end = Long.MIN_VALUE;

    private static final String LIMELIGHT_URL = "http://10.90.96.11:5800";

    private static final double DRIVE_DEADBAND = 0.05;
    private static final double ELEVATOR_DEADBAND = 0.05;

    private static final double REEF_OFFSET = 6.75; // inches
    private static final double SHIFT_SPEED = 0.125; // scale

    private static final Map<Integer, Double> reefAngles = Map.ofEntries(
        Map.entry(6, 60.0),
        Map.entry(7, 0.0),
        Map.entry(8, -60.0),
        Map.entry(9, -120.0),
        Map.entry(10, 180.0),
        Map.entry(11, 120.0),
        Map.entry(17, -60.0),
        Map.entry(18, 0.0),
        Map.entry(19, 60.0),
        Map.entry(20, 120.0),
        Map.entry(21, 180.0),
        Map.entry(22, -120.0)
    );

    public static double getOffset(double targetAngle, double heading) {
        return ((targetAngle - heading) + 180) % 360 - 180;
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

    private double getTargetAngle() {
        return (fiducialID > 0) ? reefAngles.get(fiducialID) : Double.NaN;
    }

    @Override
    public void autonomousInit() {
        var t = 3.0; // seconds
        var dx = -Units.Inches.of(36.0).in(Units.Meters);

        var xSpeed = (dx / t) / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        driveSubsystem.drive(xSpeed, 0.0, 0.0, false);

        end = System.currentTimeMillis() + (long)(t * 1000);
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
        if (shifting) {
            var now = System.currentTimeMillis();

            if (now >= end) {
                stop();

                shifting = false;
            }

            return;
        }

        var xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
        var ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

        var rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

        var aButton = driveController.getAButton();
        var bButton = driveController.getBButton();

        boolean fieldRelative;
        if (aButton || bButton) {
            xSpeed *= 0.5;
            ySpeed *= 0.5;

            if (bButton) {
                var targetAngle = getTargetAngle();

                if (Double.isNaN(targetAngle)) {
                    rot = 0.0;
                } else {
                    rot = -(getOffset(targetAngle, driveSubsystem.getHeading()) / 15.0);

                    ySpeed *= Math.max(1.0 - Math.abs(rot), 0.0) * (Math.abs(tx) / 30.0);
                }
            } else {
                rot *= 0.5;
            }

            fieldRelative = false;
        } else {
            fieldRelative = true;
        }

        driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    private void operate() {
        if (elevatorController.getAButtonPressed()) {
            elevatorSubsystem.setElevatorPosition(ElevatorPosition.RECEIVE_CORAL);
        } else if (elevatorController.getBButton()) {
            elevatorSubsystem.setElevatorSpeed(-MathUtil.applyDeadband(elevatorController.getLeftY(), ELEVATOR_DEADBAND));
        } else if (elevatorController.getLeftBumperButtonPressed()) {
            elevatorSubsystem.receiveCoral();
        } else  if (elevatorController.getRightBumperButtonPressed()) {
            elevatorSubsystem.releaseCoral();
        } else {
            var pov = elevatorController.getPOV();

            if (pov != -1) {
                switch (elevatorController.getPOV()) {
                    case 0 -> elevatorSubsystem.setElevatorPosition(ElevatorPosition.RELEASE_UPPER_CORAL);
                    case 180 -> elevatorSubsystem.setElevatorPosition(ElevatorPosition.RELEASE_LOWER_CORAL);
                    case 270 -> shift(-REEF_OFFSET);
                    case 90 -> shift(REEF_OFFSET);
                }
            }
        }
    }

    private void shift(double distance) {
        if (shifting) {
            return;
        }

        shifting = true;

        driveSubsystem.drive(0.0, -Math.signum(distance) * SHIFT_SPEED, 0.0, false);

        var dy = Units.Inches.of(distance).in(Units.Meters);
        var t = Math.abs(dy) / (SHIFT_SPEED * Constants.DriveConstants.kMaxSpeedMetersPerSecond);

        end = System.currentTimeMillis() + (long)(t * 1000);
    }

    private void stop() {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }

    @Override
    public void teleopExit() {
        stop();

        elevatorSubsystem.setElevatorPosition(ElevatorPosition.BASE);
    }
}
