package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    public enum Position {
        TRANSPORT(0.0, 0.0),
        TARGET_LOWER_TAGS(10.0, 0.0),
        TARGET_UPPER_TAGS(20.0, 0.0),
        RECEIVE_CORAL(24.0, 35.0),
        RECEIVE_LOWER_ALGAE(24.0, 90.0),
        RECEIVE_UPPER_ALGAE(36.0, 90.0),
        RELEASE_ALGAE(10.0, 135.0),
        RELEASE_LOWER_CORAL(20.0, 125.0),
        RELEASE_UPPER_CORAL(32.0, 125.0);

        private final double elevatorHeight; // inches
        private final double endEffectorAngle; // degrees

        Position(double elevatorHeight, double endEffectorAngle) {
            this.elevatorHeight = elevatorHeight;
            this.endEffectorAngle = endEffectorAngle;
        }
    }

    private SparkMax elevatorSparkMax = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
    private SparkMax endEffectorSparkMax = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
    private SparkMax intakeSparkMax = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);

    private Servo intakeServo = new Servo(0);

    private Position position = null;

    private boolean hasCoral = false;

    private static final double INITIAL_END_EFFECTOR_ANGLE = -42.5; // degrees

    private static final double ELEVATOR_DISTANCE_PER_ROTATION = 1.5; // inches
    private static final double END_EFFECTOR_DISTANCE_PER_ROTATION = 40.0; // degrees

    private static final double ALGAE_EXTRACTION_HEIGHT = 12.0; // inches
    private static final double ALGAE_EXTRACTION_ANGLE = 7.5; // degrees

    private static final double CORAL_INTAKE_POSITION = 0.75; // percent

    public ElevatorSubsystem() {
        var elevatorConfig = new SparkMaxConfig();

        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        elevatorSparkMax.configure(elevatorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        elevatorSparkMax.getEncoder().setPosition(0.0);

        var endEffectorConfig = new SparkMaxConfig();

        endEffectorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true);

        endEffectorConfig.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .p(0.18)
            .i(0)
            .d(0)
            .outputRange(-100, 100);

        endEffectorSparkMax.configure(endEffectorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        endEffectorSparkMax.getEncoder().setPosition(INITIAL_END_EFFECTOR_ANGLE / END_EFFECTOR_DISTANCE_PER_ROTATION);

        var intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(30);

        intakeSparkMax.configure(endEffectorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public void setElevatorSpeed(double speed) {
        SmartDashboard.putNumber("elevator-speed", speed);

        elevatorSparkMax.set(speed);
    }

    public void setEndEffectorSpeed(double speed) {
        speed *= -0.25;

        SmartDashboard.putNumber("end-effector-speed", speed);

        endEffectorSparkMax.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        speed *= -0.5;

        SmartDashboard.putNumber("intake-speed", speed);

        intakeSparkMax.set(speed);
    }

    public void adjustPosition(Position position) {
        this.position = position;

        setReference(position.elevatorHeight, position.endEffectorAngle);
    }

    public void extractAlgae(double time) {
        if (position != null) {
            // TODO
        }
    }

    private void setReference(double elevatorHeight, double endEffectorAngle) {
        var elevatorPosition = elevatorHeight / ELEVATOR_DISTANCE_PER_ROTATION;

        elevatorSparkMax.getClosedLoopController().setReference(elevatorPosition, SparkBase.ControlType.kPosition);

        var endEffectorPosition = endEffectorAngle / END_EFFECTOR_DISTANCE_PER_ROTATION;

        System.out.printf("endEffectorPosition = %.2f\n", endEffectorPosition);

        endEffectorSparkMax.getClosedLoopController().setReference(endEffectorPosition, SparkBase.ControlType.kPosition);
    }

    public void receiveCoral() {
        intakeServo.set(-CORAL_INTAKE_POSITION);

        hasCoral = true;
    }

    public void releaseCoral() {
        intakeServo.set(CORAL_INTAKE_POSITION);

        hasCoral = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("position", (position == null) ? "" : position.toString());

        SmartDashboard.putNumber("elevator-position", elevatorSparkMax.getEncoder().getPosition());
        SmartDashboard.putNumber("end-effector-position", endEffectorSparkMax.getEncoder().getPosition());

        SmartDashboard.putBoolean("has-coral", hasCoral);
    }
}
