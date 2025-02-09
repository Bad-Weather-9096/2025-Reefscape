package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Robot class.
 */
public class Robot extends TimedRobot {
    private XboxController driveController = new XboxController(0);
    private XboxController elevatorController = new XboxController(1);

    private DriveSubsystem driveSubsystem = new DriveSubsystem();

    private static final String LIMELIGHT_NAME = "";

    private static final double DRIVE_DEADBAND = 0.05;

    private static final String X_SPEED_KEY = "X speed";
    private static final String Y_SPEED_KEY = "Y speed";

    private static final String ROTATION_KEY = "Rotation";

    private static final String TX_KEY = "TX";
    private static final String TY_KEY = "TY";
    private static final String TA_KEY = "TA";
    private static final String TV_KEY = "TV";

    private static final String ID_KEY = "ID";
    private static final String CAMERA_DISTANCE_KEY = "Distance to Camera";
    private static final String ROBOT_DISTANCE_KEY = "Distance to Robot";

    @Override
    @SuppressWarnings("resource")
    public void robotInit() {
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void teleopPeriodic() {
        if (elevatorController.getXButton()) {
            SmartDashboard.putNumber(X_SPEED_KEY, 0.0);
            SmartDashboard.putNumber(Y_SPEED_KEY, 0.0);

            SmartDashboard.putNumber(ROTATION_KEY, 0.0);

            driveSubsystem.setX();
        } else {
            var xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
            var ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

            SmartDashboard.putNumber(X_SPEED_KEY, xSpeed);
            SmartDashboard.putNumber(Y_SPEED_KEY, ySpeed);

            var rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

            SmartDashboard.putNumber(ROTATION_KEY, rot);

            driveSubsystem.drive(xSpeed, ySpeed, rot, !driveController.getYButton());
        }

        SmartDashboard.putNumber(TX_KEY, LimelightHelpers.getTX(LIMELIGHT_NAME));
        SmartDashboard.putNumber(TY_KEY, LimelightHelpers.getTY(LIMELIGHT_NAME));
        SmartDashboard.putNumber(TA_KEY, LimelightHelpers.getTA(LIMELIGHT_NAME));

        SmartDashboard.putBoolean(TV_KEY, LimelightHelpers.getTV(LIMELIGHT_NAME));

        var rawFiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_NAME);

        if (rawFiducials.length == 1) {
            var rawFiducial = rawFiducials[0];

            SmartDashboard.putNumber(ID_KEY, rawFiducial.id);
            SmartDashboard.putNumber(CAMERA_DISTANCE_KEY, rawFiducial.distToCamera);
            SmartDashboard.putNumber(ROBOT_DISTANCE_KEY, rawFiducial.distToRobot);
        } else {
            SmartDashboard.putNumber(ID_KEY, 0);
            SmartDashboard.putNumber(CAMERA_DISTANCE_KEY, 0.0);
            SmartDashboard.putNumber(ROBOT_DISTANCE_KEY, 0.0);
        }
    }
}
