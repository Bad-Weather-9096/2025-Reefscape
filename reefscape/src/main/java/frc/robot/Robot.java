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

    private long end = Long.MIN_VALUE;

    private static final String LIMELIGHT_URL = "http://10.90.96.11:5800";

    private static final double DRIVE_DEADBAND = 0.05;
    private static final double ELEVATOR_DEADBAND = 0.05;

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

    private static double normalizeAngle(double angle) {
        return (angle + 180.0) % 360.0 - 180.0;
    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture(new HttpCamera("limelight", LIMELIGHT_URL, HttpCameraKind.kMJPGStreamer));
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("heading", driveSubsystem.getHeading());

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

        SmartDashboard.putNumber("target-angle", getTargetAngle());
    }

    private double getTargetAngle() {
        return (fiducialID > 0) ? reefAngles.get(fiducialID - 1) : Double.NaN;
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
            driveSubsystem.drive(0.0, 0.0, 0.0, false);
        }
    }

    @Override
    public void teleopPeriodic() {
        readLimelight();

        navigate();
        operate();
    }

    private void navigate() {
        var xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
        var ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

        var rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

        boolean fieldRelative;
        if (driveController.getAButton()) {
            xSpeed *= 0.25;
            ySpeed *= 0.25;

            rot *= 0.5;

            fieldRelative = false;
        } else if (driveController.getBButton()) {
            xSpeed *= 0.25;

            var targetAngle = getTargetAngle();

            if (Double.isNaN(targetAngle)) {
                ySpeed *= 0.25;

                rot = 0.0;
            } else {
                var offset = normalizeAngle(targetAngle) - normalizeAngle(driveSubsystem.getHeading());

                rot = -offset / 15.0;

                ySpeed *= Math.max(1.0 - Math.abs(rot), 0.0) * (Math.abs(tx) / 30.0);
            }

            fieldRelative = false;
        } else {
            tx = 0.0;

            fiducialID = -1;

            fieldRelative = true;
        }

        driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    private void operate() {
        if (elevatorController.getAButtonPressed()) {
            elevatorSubsystem.setElevatorPosition(ElevatorPosition.RECEIVE_CORAL);
        } else if (elevatorController.getBButton()) {
            switch (elevatorController.getPOV()) {
                case 0 -> elevatorSubsystem.setElevatorPosition(ElevatorPosition.RELEASE_UPPER_CORAL);
                case 180 -> elevatorSubsystem.setElevatorPosition(ElevatorPosition.RELEASE_LOWER_CORAL);
            }
        } else {
            var elevatorSpeed = -MathUtil.applyDeadband(elevatorController.getLeftY(), ELEVATOR_DEADBAND);

            if (elevatorSpeed != 0.0) {
                elevatorSubsystem.setElevatorSpeed(elevatorSpeed);
            }
        }
    }

    @Override
    public void teleopExit() {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);

        elevatorSubsystem.setElevatorPosition(ElevatorPosition.BASE);
    }
}
