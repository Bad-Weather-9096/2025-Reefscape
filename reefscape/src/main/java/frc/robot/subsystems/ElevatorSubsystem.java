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
        BASE(0.0, 0.0),
        TARGET_LOWER_TAGS(0.0, 0.0), // TODO
        TARGET_UPPER_TAGS(0.0, 0.0), // TODO
        RECEIVE_CORAL(0.0, 0.0), // TODO
        RELEASE_LOWER_CORAL(0.0, 0.0), // TODO
        RELEASE_UPPER_CORAL(0.0, 0.0), // TODO
        RECEIVE_LOWER_ALGAE(0.0, 0.0), // TODO
        RECEIVE_UPPER_ALGAE(0.0, 0.0), // TODO
        RELEASE_ALGAE(0.0, 0.0); // TODO

        private final double elevatorPosition;
        private final double endEffectorPosition;

        Position(double elevatorPosition, double endEffectorPosition) {
            this.elevatorPosition = elevatorPosition;
            this.endEffectorPosition = endEffectorPosition;
        }
    }

    private SparkMax elevatorSparkMax = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
    private SparkMax endEffectorSparkMax = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);

    private Servo coralIntakeServo = new Servo(0);

    private SparkMax algaeIntakeSparkMax = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);

    private Position position = null;

    private boolean hasCoral = false;

    private static final double CORAL_INTAKE_POSITION = 0.75; // percent

    public ElevatorSubsystem() {
        // Elevator
        var elevatorConfig = new SparkMaxConfig();

        elevatorConfig.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1);

        elevatorSparkMax.configure(elevatorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        elevatorSparkMax.getEncoder().setPosition(0);

        // End effector
        var endEffectorConfig = new SparkMaxConfig();

        endEffectorConfig.inverted(true);

        endEffectorConfig.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1);

        endEffectorSparkMax.configure(endEffectorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        endEffectorSparkMax.getEncoder().setPosition(0);

        // Intake
        var intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(30);

        algaeIntakeSparkMax.configure(intakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public void setElevatorSpeed(double speed) {
        elevatorSparkMax.set(speed);
    }

    public void setEndEffectorPosition(double position) {
        endEffectorSparkMax.getClosedLoopController().setReference(position * 45.0, SparkBase.ControlType.kPosition);
    }

    public void setIntakeSpeed(double speed) {
        algaeIntakeSparkMax.set(0.5 * speed);
    }

    public void setPosition(Position position) {
        if (position == null) {
            throw new IllegalArgumentException();
        }

        this.position = position;

        elevatorSparkMax.getClosedLoopController().setReference(position.elevatorPosition, SparkBase.ControlType.kPosition);
        endEffectorSparkMax.getClosedLoopController().setReference(position.endEffectorPosition, SparkBase.ControlType.kPosition);
    }

    public void receiveCoral() {
        coralIntakeServo.set(-CORAL_INTAKE_POSITION);

        hasCoral = true;
    }

    public void releaseCoral() {
        coralIntakeServo.set(CORAL_INTAKE_POSITION);

        hasCoral = false;
    }

    public void receiveAlgae() {
        algaeIntakeSparkMax.set(0.5);
    }

    public void releaseAlgae() {
        algaeIntakeSparkMax.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("position", (position == null) ? "" : position.toString());

        SmartDashboard.putNumber("elevator-position", elevatorSparkMax.getEncoder().getPosition());
        SmartDashboard.putNumber("end-effector-position", endEffectorSparkMax.getEncoder().getPosition());

        SmartDashboard.putNumber("coral-intake-position", coralIntakeServo.get());

        SmartDashboard.putNumber("algae-intake-speed", algaeIntakeSparkMax.get());

        SmartDashboard.putBoolean("has-coral", hasCoral);
    }
}
