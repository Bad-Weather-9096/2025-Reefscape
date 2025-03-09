package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    public enum Position {
        TARGET_UPPER_TAGS(20.0, 0.0),
        TARGET_LOWER_TAGS(10.0, 0.0),
        RECEIVE_CORAL(24.0, 45.0),
        RECEIVE_LOWER_ALGAE(32.0, 90.0),
        RECEIVE_UPPER_ALGAE(48.0, 90.0),
        RELEASE_ALGAE(10.0, 135.0),
        RELEASE_LOWER_CORAL(40.0, 125.0),
        RELEASE_UPPER_CORAL(56.0, 125.0);

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

    private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(0.2, 1.40, 3.07, 0.18);
    private ArmFeedforward endEffectorFeedForward = new ArmFeedforward(0.2, 0.56, 1.95, 0.04);

    // TODO
    private static final double ELEVATOR_DISTANCE_PER_ROTATION = 1.0; // inches

    public ElevatorSubsystem() {
        var elevatorConfig = new SparkMaxConfig();

        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        elevatorSparkMax.configure(elevatorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        elevatorSparkMax.getEncoder().setPosition(0.0);

        var endEffectorConfig = new SparkMaxConfig();

        endEffectorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        endEffectorSparkMax.configure(endEffectorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        endEffectorSparkMax.getEncoder().setPosition(0.0);

        var intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(30);

        intakeSparkMax.configure(endEffectorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);
    }

    public void setElevatorSpeed(double speed) {
        SmartDashboard.putNumber("elevator-speed", speed);

        elevatorSparkMax.set(speed);
    }

    public void setEndEffectorSpeed(double speed) {
        SmartDashboard.putNumber("end-effector-speed", speed);

        endEffectorSparkMax.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        SmartDashboard.putNumber("intake-speed", speed);

        intakeSparkMax.set(speed);
    }

    public void receiveCoral() {
        // TODO
    }

    public void releaseCoral() {
        // TODO
    }

    public void setPosition(Position position) {
        var elevatorPosition = position.elevatorHeight / ELEVATOR_DISTANCE_PER_ROTATION;

        SmartDashboard.putNumber("elevator-position", elevatorPosition);

        // TODO Velocity setpoint is rotations/second
        elevatorSparkMax.getClosedLoopController().setReference(elevatorPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            elevatorFeedForward.calculate(0.0));

        var endEffectorPosition = position.endEffectorAngle;

        SmartDashboard.putNumber("end-effector-position", endEffectorPosition);

        // TODO positionRadians is angle relative to zero-horizontal
        // TODO Velocity setpoint is rotations/second
        endEffectorSparkMax.getClosedLoopController().setReference(endEffectorPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            endEffectorFeedForward.calculate(0.0, 0.0));
    }
}
