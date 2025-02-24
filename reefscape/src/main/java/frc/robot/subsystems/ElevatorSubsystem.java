package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    public enum Position {
        // TODO
        BASE(0.0, 0.0),
        PROCESSOR(0.0, 0.0),
        CORAL_INTAKE(0.0, 0.0),
        LOWER_ALGAE(0.0, 0.0),
        UPPER_ALGAE(0.0, 0.0),
        LOWER_CORAL(0.0, 0.0),
        UPPER_CORAL(0.0, 0.0);

        private final double elevatorExtension; // inches
        private final double endEffectorAngle; // degrees

        Position(double elevatorExtension, double endEffectorAngle) {
            this.elevatorExtension = elevatorExtension;
            this.endEffectorAngle = endEffectorAngle;
        }
    }

    private SparkMax elevatorController;
    private SparkMax endEffectorController;

    private SparkMax algaeIntakeController;

    private Servo coralIntakeServo;

    private Position position = null;

    private boolean hasCoral = false;

    private static final int ELEVATOR_CAN_ID = 9;
    private static final int END_EFFECTOR_CAN_ID = 10;

    private static final int ALGAE_INTAKE_CAN_ID = 11;

    private static final double CAMERA_OFFSET = 4.25; // inches

    // TODO Elevator constants
    private static final int TICKS_PER_INCH = 24;

    private static final double ELEVATOR_SPEED = 0.2; // percent
    private static final double MAXIMUM_ELEVATOR_EXTENSION = 24.0; // inches
    private static final double ELEVATOR_EXTENSION_DEADBAND = 0.1;

    // TODO End effector constants
    private static final int TICKS_PER_DEGREE = 24;

    private static final double END_EFFECTOR_SPEED = 0.1; // percent
    private static final double MAXIMUM_END_EFFECTOR_ROTATION = 270.0; // degrees
    private static final double END_EFFECTOR_ANGLE_DEADBAND = 0.1;

    // TODO Algae intake constants
    private static final double ALGAE_INTAKE_SPEED = 0.1; // percent

    // TODO Coral intake constants
    private static final double CORAL_INTAKE_POSITION = 0.5;

    public ElevatorSubsystem() {
        elevatorController = new SparkMax(ELEVATOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var elevatorConfig = new SparkMaxConfig();

        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        elevatorController.configure(elevatorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        elevatorController.getEncoder().setPosition(0);

        endEffectorController = new SparkMax(END_EFFECTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var endEffectorConfig = new SparkMaxConfig();

        endEffectorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        endEffectorController.configure(endEffectorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        endEffectorController.getEncoder().setPosition(0);

        algaeIntakeController = new SparkMax(ALGAE_INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var algaeIntakeConfig = new SparkMaxConfig();

        algaeIntakeConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        algaeIntakeController.configure(algaeIntakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        coralIntakeServo = new Servo(0);
    }

    public double getCameraHeight() {
        return getElevatorExtension() + CAMERA_OFFSET;
    }

    public double getElevatorExtension() {
        return elevatorController.getEncoder().getPosition() / TICKS_PER_INCH;
    }

    public void raiseElevator() {
        position = null;

        elevatorController.set(getElevatorExtension() < MAXIMUM_ELEVATOR_EXTENSION ? ELEVATOR_SPEED : 0.0);
    }

    public void lowerElevator() {
        position = null;

        elevatorController.set(getElevatorExtension() > 0 ? -ELEVATOR_SPEED : 0.0);
    }

    public void stopElevator() {
        if (position == null) {
            elevatorController.set(0.0);
        }
    }

    public double getEndEffectorAngle() {
        return endEffectorController.getEncoder().getPosition() / TICKS_PER_DEGREE;
    }

    public void raiseEndEffector() {
        position = null;

        endEffectorController.set(getEndEffectorAngle() > 0 ? -END_EFFECTOR_SPEED : 0.0);
    }

    public void lowerEndEffector() {
        position = null;

        endEffectorController.set(getEndEffectorAngle() < MAXIMUM_END_EFFECTOR_ROTATION ? END_EFFECTOR_SPEED : 0.0);
    }

    public void stopEndEffector() {
        if (position == null) {
            endEffectorController.set(0.0);
        }
    }

    public void receiveCoral() {
        coralIntakeServo.set(CORAL_INTAKE_POSITION);

        hasCoral = true;
    }

    public void releaseCoral() {
        coralIntakeServo.set(-CORAL_INTAKE_POSITION);

        hasCoral = false;
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public void receiveAlgae() {
        algaeIntakeController.set(ALGAE_INTAKE_SPEED);
    }

    public void releaseAlgae() {
        // TODO Reverse briefly?
        algaeIntakeController.set(0.0);
    }

    public void adjustPosition(Position position) {
        this.position = position;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("camera-height", getCameraHeight());

        var elevatorExtension = getElevatorExtension();

        if (position != null) {
            var elevatorExtensionDelta = MathUtil.applyDeadband(position.elevatorExtension - elevatorExtension, ELEVATOR_EXTENSION_DEADBAND);
            var elevatorSpeed = Math.signum(elevatorExtensionDelta) * ELEVATOR_SPEED;

            endEffectorController.set(elevatorSpeed);
        }

        SmartDashboard.putNumber("elevator-extension", elevatorExtension);

        var endEffectorAngle = getEndEffectorAngle();

        if (position != null) {
            var endEffectorAngleDelta = MathUtil.applyDeadband(position.endEffectorAngle - endEffectorAngle, END_EFFECTOR_ANGLE_DEADBAND);
            var endEffectorSpeed = Math.signum(endEffectorAngleDelta) * END_EFFECTOR_SPEED;

            endEffectorController.set(endEffectorSpeed);
        }

        SmartDashboard.putNumber("end-effector-angle", endEffectorAngle);

        SmartDashboard.putBoolean("has-coral", hasCoral);
    }
}
