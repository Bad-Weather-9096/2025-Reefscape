package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

/**
 * Implements robot functionality.
 */
public class Robot extends TimedRobot {
    private XboxController driveController = new XboxController(0);
    private XboxController auxilliaryController = new XboxController(1);

    private DriveSubsystem driveSubsystem = new DriveSubsystem();

    private AutoPilotParameters autoPilotParameters = null;

    private static final String LIMELIGHT_NAME = "";

    private static final String TV_KEY = "tv";
    private static final String TX_KEY = "tx";
    private static final String TY_KEY = "ty";

    private static final String FIDUCIAL_ID_KEY = "fiducial-id";

    private static final String X_SPEED_KEY = "x-speed";
    private static final String Y_SPEED_KEY = "y-Speed";

    private static final String ROT_KEY = "rot";

    private static final double DRIVE_DEADBAND = 0.05;

    private static final double KP_RANGE = 0.1;
    private static final double KP_AIM = 0.035;

    private static final Distance cameraHeight = Distance.ofBaseUnits(9.25, Units.Inches);

    private static final List<FieldElement> fieldElements = List.of(
        new FieldElement(1, FieldElement.Type.CORAL_STATION, Angle.ofBaseUnits(-126.0, Units.Degrees)),
        new FieldElement(2, FieldElement.Type.CORAL_STATION, Angle.ofBaseUnits(126.0, Units.Degrees)),
        new FieldElement(3, FieldElement.Type.PROCESSOR, Angle.ofBaseUnits(90.0, Units.Degrees)),
        new FieldElement(4, FieldElement.Type.BARGE, Angle.ofBaseUnits(0.0, Units.Degrees)),
        new FieldElement(5, FieldElement.Type.BARGE, Angle.ofBaseUnits(0.0, Units.Degrees)),
        new FieldElement(6, FieldElement.Type.REEF, Angle.ofBaseUnits(60.0, Units.Degrees)),
        new FieldElement(7, FieldElement.Type.REEF, Angle.ofBaseUnits(0.0, Units.Degrees)),
        new FieldElement(8, FieldElement.Type.REEF, Angle.ofBaseUnits(-60.0, Units.Degrees)),
        new FieldElement(9, FieldElement.Type.REEF, Angle.ofBaseUnits(-120.0, Units.Degrees)),
        new FieldElement(10, FieldElement.Type.REEF, Angle.ofBaseUnits(180.0, Units.Degrees)),
        new FieldElement(11, FieldElement.Type.REEF, Angle.ofBaseUnits(120.0, Units.Degrees)),
        new FieldElement(12, FieldElement.Type.CORAL_STATION, Angle.ofBaseUnits(126.0, Units.Degrees)),
        new FieldElement(13, FieldElement.Type.CORAL_STATION, Angle.ofBaseUnits(-126.0, Units.Degrees)),
        new FieldElement(14, FieldElement.Type.BARGE, Angle.ofBaseUnits(0.0, Units.Degrees)),
        new FieldElement(15, FieldElement.Type.BARGE, Angle.ofBaseUnits(0.0, Units.Degrees)),
        new FieldElement(16, FieldElement.Type.PROCESSOR, Angle.ofBaseUnits(90.0, Units.Degrees)),
        new FieldElement(17, FieldElement.Type.REEF, Angle.ofBaseUnits(-60.0, Units.Degrees)),
        new FieldElement(18, FieldElement.Type.REEF, Angle.ofBaseUnits(0.0, Units.Degrees)),
        new FieldElement(19, FieldElement.Type.REEF, Angle.ofBaseUnits(60.0, Units.Degrees)),
        new FieldElement(20, FieldElement.Type.REEF, Angle.ofBaseUnits(120.0, Units.Degrees)),
        new FieldElement(21, FieldElement.Type.REEF, Angle.ofBaseUnits(180.0, Units.Degrees)),
        new FieldElement(22, FieldElement.Type.REEF, Angle.ofBaseUnits(-120.0, Units.Degrees))
    );

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
        var tv = LimelightHelpers.getTV(LIMELIGHT_NAME);

        var tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        var ty = LimelightHelpers.getTY(LIMELIGHT_NAME);

        var fiducialID = LimelightHelpers.getFiducialID(LIMELIGHT_NAME);

        SmartDashboard.putBoolean(TV_KEY, tv);

        SmartDashboard.putNumber(TX_KEY, tx);
        SmartDashboard.putNumber(TY_KEY, ty);

        SmartDashboard.putNumber(FIDUCIAL_ID_KEY, fiducialID);

        if (auxilliaryController.getXButton()) {
            block();
        } else {
            drive(tv, tx, ty, fiducialID);
        }

        driveSubsystem.periodic();
    }

    private void block() {
        SmartDashboard.putNumber(X_SPEED_KEY, 0.0);
        SmartDashboard.putNumber(Y_SPEED_KEY, 0.0);

        SmartDashboard.putNumber(ROT_KEY, 0.0);

        driveSubsystem.setX();
    }

    private void drive(boolean tv, double tx, double ty, double fiducialID) {
        double xSpeed;
        double ySpeed;
        double rot;
        boolean fieldRelative;
        if (driveController.getAButton() && tv) {
            if (autoPilotParameters == null) {
                var fieldElement = fieldElements.get((int)fiducialID - 1);

                var ht = fieldElement.type().getHeight().in(Units.Meters);
                var hc = cameraHeight.in(Units.Meters);

                var dx = (ht - hc) / Math.tan(Math.toRadians(ty));
                var dy = dx * Math.tan(Math.toRadians(tx));

                var angle = fieldElement.angle().in(Units.Radians);
                var heading = Math.toRadians(driveSubsystem.getHeading());

                var a = angle - heading;

                var t = getTime(dx, dy, a);

                var start = System.currentTimeMillis();

                autoPilotParameters = new AutoPilotParameters(start + (long)(t * 1000), dx / t, dy / t, a / t);
            }

            var now = System.currentTimeMillis();

            if (now < autoPilotParameters.end()) {
                xSpeed = autoPilotParameters.xSpeed();
                ySpeed = autoPilotParameters.ySpeed();

                rot = autoPilotParameters.rot();
            } else {
                xSpeed = 0.0;
                ySpeed = 0.0;

                rot = 0.0;
            }

            fieldRelative = false;
        } else {
            autoPilotParameters = null;

            xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
            ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

            rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

            fieldRelative = true;

            if (driveController.getYButton()) {
                xSpeed = -ty * KP_RANGE;

                rot = -tx * KP_AIM;

                fieldRelative = false;
            }
        }

        SmartDashboard.putNumber(X_SPEED_KEY, xSpeed);
        SmartDashboard.putNumber(Y_SPEED_KEY, ySpeed);

        SmartDashboard.putNumber(ROT_KEY, rot);

        driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    private double getTime(double dx, double dy, double a) {
        var tx = dx / Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        var ty = dy / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        var ta = Math.abs(a) / Constants.DriveConstants.kMaxAngularSpeed;

        return Math.max(Math.max(tx, ty), ta);
    }
}
