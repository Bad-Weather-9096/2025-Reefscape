package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
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
    private XboxController auxilliaryController = new XboxController(1);

    private DriveSubsystem driveSubsystem = new DriveSubsystem();

    private static final String LIMELIGHT_NAME = "";

    private static final String TX_KEY = "TX";
    private static final String TY_KEY = "TY";
    private static final String TA_KEY = "TA";

    private static final String TV_KEY = "TV";

    private static final String FIDUCIAL_ID_KEY = "Fiducial ID";

    private static final String X_SPEED_KEY = "X speed";
    private static final String Y_SPEED_KEY = "Y speed";

    private static final String ROTATION_KEY = "Rotation";

    private static final double DRIVE_DEADBAND = 0.02;

    private static final double KP_RANGE = 0.1;
    private static final double KP_AIM = 0.035;

    @Override
    public void robotInit() {
        var limelightFeed = new HttpCamera("limelight", "http://10.90.96.11:5800", HttpCameraKind.kMJPGStreamer);

        CameraServer.startAutomaticCapture(limelightFeed);
    }

    @Override
    public void autonomousPeriodic() {
        driveSubsystem.periodic();
    }

    @Override
    public void teleopPeriodic() {
        var tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        var ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
        var ta = LimelightHelpers.getTA(LIMELIGHT_NAME);

        var tv = LimelightHelpers.getTV(LIMELIGHT_NAME);

        var fiducialID = LimelightHelpers.getFiducialID(LIMELIGHT_NAME);

        SmartDashboard.putNumber(TX_KEY, tx);
        SmartDashboard.putNumber(TY_KEY, ty);
        SmartDashboard.putNumber(TA_KEY, ta);

        SmartDashboard.putBoolean(TV_KEY, tv);

        SmartDashboard.putNumber(FIDUCIAL_ID_KEY, fiducialID);

        if (auxilliaryController.getXButton()) {
            SmartDashboard.putNumber(X_SPEED_KEY, 0.0);
            SmartDashboard.putNumber(Y_SPEED_KEY, 0.0);

            SmartDashboard.putNumber(ROTATION_KEY, 0.0);

            driveSubsystem.setX();
        } else {
            var xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
            var ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

            var rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

            var fieldRelative = true;

            if (tv && driveController.getAButton()) {
                xSpeed = -ty * KP_RANGE;

                rot = -tx * KP_AIM;

                fieldRelative = false;
            }

            SmartDashboard.putNumber(X_SPEED_KEY, xSpeed);
            SmartDashboard.putNumber(Y_SPEED_KEY, ySpeed);

            SmartDashboard.putNumber(ROTATION_KEY, rot);

            driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
        }
    }
}
