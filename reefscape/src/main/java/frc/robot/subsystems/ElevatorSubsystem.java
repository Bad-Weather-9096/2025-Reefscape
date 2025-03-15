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
        START(0.0, 1.5),
        TARGET_LOWER_TAGS(62.0, 1.5),
        TARGET_UPPER_TAGS(112.0, 1.5),
        RECEIVE_CORAL(102.0, 1.5),
        RELEASE_LOWER_CORAL(130.0, 4.0),
        RELEASE_UPPER_CORAL(185.0, 4.0),
        RECEIVE_LOWER_ALGAE(130.0, 4.0), // TODO
        RECEIVE_UPPER_ALGAE(185.0, 4.0), // TODO
        RELEASE_ALGAE(0.0, 3.0);

        private final double elevatorPosition;
        private final double endEffectorPosition;

        Position(double elevatorPosition, double endEffectorPosition) {
            this.elevatorPosition = elevatorPosition;
            this.endEffectorPosition = endEffectorPosition;
        }
    }

    private SparkMax elevatorSparkMax = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
    private SparkMax endEffectorSparkMax = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);

    private SparkMax algaeIntakeSparkMax = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);

    private Servo coralIntakeServo = new Servo(0);

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

    public void setPosition(Position position) {
        elevatorSparkMax.getClosedLoopController().setReference(position.elevatorPosition, SparkBase.ControlType.kPosition);
        endEffectorSparkMax.getClosedLoopController().setReference(position.endEffectorPosition, SparkBase.ControlType.kPosition);
    }

    public void setElevatorSpeed(double speed) {
        elevatorSparkMax.set(speed);
    }

    public void setEndEffectorSpeed(double speed) {
        endEffectorSparkMax.set(speed);
    }

    public void setAlgaeIntakeSpeed(double speed) {
        algaeIntakeSparkMax.set(speed);
    }

    public void receiveCoral() {
        coralIntakeServo.set(-CORAL_INTAKE_POSITION);
    }

    public void releaseCoral() {
        coralIntakeServo.set(CORAL_INTAKE_POSITION);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator-position", elevatorSparkMax.getEncoder().getPosition());
        SmartDashboard.putNumber("end-effector-position", endEffectorSparkMax.getEncoder().getPosition());

        SmartDashboard.putNumber("coral-intake-position", coralIntakeServo.get());

        SmartDashboard.putNumber("algae-intake-speed", algaeIntakeSparkMax.get());
    }
}
