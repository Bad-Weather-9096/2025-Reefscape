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

    private double fiducialID = -1.0;

    private AutonomousMode autonomousMode = null;

    private AutoPilotParameters autoPilotParameters = null;

    private static final String LIMELIGHT_URL = "http://10.90.96.11:5800";

    private static final String LIMELIGHT_NAME = "";

    private static final String TV_KEY = "tv";
    private static final String TX_KEY = "tx";
    private static final String TY_KEY = "ty";

    private static final String FIDUCIAL_ID_KEY = "fiducial-id";

    private static final double REVERSE_DISTANCE = 72.0; // inches
    private static final double REVERSE_TIME = 2.0; // seconds

    private static final double LOCATE_TAG_SPEED = Math.PI / 2; // radians/second

    private static final double DRIVE_DEADBAND = 0.05;

    private static final double ELEVATOR_DEADBAND = 0.02;
    private static final double END_EFFECTOR_DEADBAND = 0.02;

    private static final double ALGAE_DEADBAND = 0.05;

    private static final double BASE_HEIGHT = 5.0; // inches

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

        fiducialID = LimelightHelpers.getFiducialID(LIMELIGHT_NAME);

        SmartDashboard.putBoolean(TV_KEY, tv);

        SmartDashboard.putNumber(TX_KEY, tx);
        SmartDashboard.putNumber(TY_KEY, ty);

        SmartDashboard.putNumber(FIDUCIAL_ID_KEY, fiducialID);
    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture(new HttpCamera("limelight", LIMELIGHT_URL, HttpCameraKind.kMJPGStreamer));
    }

    @Override
    public void autonomousInit() {
        autonomousMode = null;

        autoPilotParameters = null;
    }

    @Override
    public void autonomousPeriodic() {
        readLimelight();

        var now = System.currentTimeMillis();

        if (autonomousMode == null) {
            autonomousMode = AutonomousMode.REVERSE;

            var dr = Distance.ofBaseUnits(REVERSE_DISTANCE, Units.Inches).in(Units.Meters);

            var xSpeed = -(dr / REVERSE_TIME) / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

            autoPilotParameters = new AutoPilotParameters(now + (long)(REVERSE_TIME * 1000), xSpeed, 0.0, 0.0);
        } else {
            switch (autonomousMode) {
                case REVERSE -> {
                    if (now >= autoPilotParameters.end()) {
                        autonomousMode = AutonomousMode.LOCATE_TAG;

                        var rot = LOCATE_TAG_SPEED / Constants.DriveConstants.kMaxAngularSpeed;

                        autoPilotParameters = new AutoPilotParameters(0, 0.0, 0.0, rot);
                    }
                }
                case LOCATE_TAG -> {
                    if (tv) {
                        var fieldElement = fieldElements.get((int)fiducialID - 1);

                        if (fieldElement.getType() == FieldElement.Type.REEF) {
                            autonomousMode = AutonomousMode.DOCK;

                            autoPilotParameters = getDockingParameters();
                        }
                    }
                }
                case DOCK -> {
                    if (now >= autoPilotParameters.end()) {
                        autonomousMode = AutonomousMode.DONE;

                        autoPilotParameters = new AutoPilotParameters(0, 0.0, 0.0, 0.0);
                    }
                }
                case DONE -> {
                    // No-op
                }
            }
        }

        driveSubsystem.drive(autoPilotParameters.xSpeed(), autoPilotParameters.ySpeed(), autoPilotParameters.rot(), false);

        driveSubsystem.periodic();

        elevatorSubsystem.periodic();
    }

    @Override
    public void autonomousExit() {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }

    @Override
    public void teleopInit() {
        autoPilotParameters = null;
    }

    @Override
    public void teleopPeriodic() {
        readLimelight();

        drive();
        operate();
    }

    private void drive() {
        double xSpeed;
        double ySpeed;
        double rot;
        boolean fieldRelative;
        if (auxilliaryController.getAButton() && tv) {
            if (autoPilotParameters == null) {
                autoPilotParameters = getDockingParameters();
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
        }

        driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);

        driveSubsystem.periodic();
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
        }

        if (auxilliaryController.getRightBumperButtonPressed()) {
            elevatorSubsystem.releaseCoral();
        }

        if (MathUtil.applyDeadband(auxilliaryController.getLeftTriggerAxis(), ALGAE_DEADBAND) > 0.0) {
            elevatorSubsystem.receiveAlgae();
        } else if (MathUtil.applyDeadband(auxilliaryController.getRightTriggerAxis(), ALGAE_DEADBAND) > 0.0) {
            elevatorSubsystem.releaseAlgae();
        } else {
            elevatorSubsystem.stopAlgae();
        }

        var pov = auxilliaryController.getPOV();

        if (pov != -1) {
            var direction = Direction.fromAngle(pov);

            var fieldElement = fieldElements.get((int)fiducialID - 1);

            switch (fieldElement.getType()) {
                case CORAL_STATION -> {
                    switch (direction) {
                        case LEFT -> {
                            // TODO Move to left slot
                        }
                        case RIGHT -> {
                            // TODO Move to right slot
                        }
                    }
                }
                case REEF -> {
                    switch (direction) {
                        case UP -> {
                            // TODO If carrying coral, adjust height for upper coral release; otherwise, for upper algae intake
                        }
                        case DOWN -> {
                            // TODO If carrying coral, adjust height for lower coral release; otherwise, for lower algae intake
                        }
                        case LEFT -> {
                            // TODO If carrying coral, move to left branch
                        }
                        case RIGHT -> {
                            // TODO If carrying coral, move to right branch
                        }
                    }
                }
            }
        }

        elevatorSubsystem.periodic();
    }

    @Override
    public void teleopExit() {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }

    private AutoPilotParameters getDockingParameters() {
        var fieldElement = fieldElements.get((int)fiducialID - 1);

        var ht = fieldElement.getType().getHeight().in(Units.Meters);
        var hc = Distance.ofBaseUnits(BASE_HEIGHT + elevatorSubsystem.getCameraHeight(), Units.Inches).in(Units.Meters);

        var dx = (ht - hc) / Math.tan(Math.toRadians(ty));
        var dy = dx * Math.tan(Math.toRadians(tx));

        var angle = fieldElement.getAngle().in(Units.Radians);
        var heading = Math.toRadians(driveSubsystem.getHeading());

        var a = angle - heading;

        var t = getTime(dx, dy, a);

        var now = System.currentTimeMillis();

        return new AutoPilotParameters(now + (long)(t * 1000), dx / t, dy / t, a / t);
    }

    private double getTime(double dx, double dy, double a) {
        var tx = dx / Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        var ty = dy / Constants.DriveConstants.kMaxSpeedMetersPerSecond;

        var ta = Math.abs(a) / Constants.DriveConstants.kMaxAngularSpeed;

        return Math.max(Math.max(tx, ty), ta) * 2.0;
    }
}
