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
    private static final double ELEVATOR_DEADBAND = 0.075;
    private static final double END_EFFECTOR_DEADBAND = 0.075;

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
    public void teleopInit() {
        elevatorSubsystem.setPosition(ElevatorSubsystem.Position.START);
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
        } else {
            fieldRelative = true;
        }

        driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    private void operate() {
        if (elevatorController.getAButtonPressed()) {
            elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RECEIVE_CORAL);
        } else if (elevatorController.getBButton()) {
            switch (elevatorController.getPOV()) {
                case 0 -> elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RELEASE_UPPER_CORAL);
                case 180 -> elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RELEASE_LOWER_CORAL);
            }
        } else if (elevatorController.getXButtonPressed()) {
            elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RELEASE_ALGAE);
        } else if (elevatorController.getYButton()) {
            switch (elevatorController.getPOV()) {
                case 0 -> elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RECEIVE_UPPER_ALGAE);
                case 180 -> elevatorSubsystem.setPosition(ElevatorSubsystem.Position.RECEIVE_LOWER_ALGAE);
            }
        } else if (elevatorController.getLeftBumperButtonPressed() && elevatorController.getRightBumperButtonPressed()) {
            elevatorSubsystem.setPosition(ElevatorSubsystem.Position.BASE);
        } else if (elevatorController.getLeftBumperButtonPressed()) {
            elevatorSubsystem.receiveCoral();
        } else if (elevatorController.getRightBumperButtonPressed()) {
            elevatorSubsystem.releaseCoral();
        } else {
            var elevatorSpeed = -MathUtil.applyDeadband(elevatorController.getLeftY(), ELEVATOR_DEADBAND);

            if (elevatorSpeed != 0.0) {
                elevatorSubsystem.setElevatorSpeed(elevatorSpeed);
            }

            var endEffectorSpeed = -MathUtil.applyDeadband(elevatorController.getRightY(), END_EFFECTOR_DEADBAND);

            if (endEffectorSpeed != 0.0) {
                elevatorSubsystem.setEndEffectorSpeed(endEffectorSpeed * 0.15);
            }

            var leftTriggerAxis = elevatorController.getLeftTriggerAxis();
            var rightTriggerAxis = elevatorController.getRightTriggerAxis();

            elevatorSubsystem.setAlgaeIntakeSpeed(rightTriggerAxis - leftTriggerAxis);
        }
    }

    @Override
    public void teleopExit() {
        stop();

        elevatorSubsystem.setPosition(ElevatorSubsystem.Position.BASE);
    }

    private void stop() {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }
}
