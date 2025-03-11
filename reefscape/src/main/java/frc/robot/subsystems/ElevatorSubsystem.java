package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    public enum Position {
        TARGET_LOWER_TAGS(10.0, 0.0),
        TARGET_UPPER_TAGS(20.0, 0.0),
        RECEIVE_CORAL(24.0, 45.0),
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

    private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(0.2, 0.81, 3.07, 0.10);
    private ArmFeedforward endEffectorFeedForward = new ArmFeedforward(0.2, 0.75, 0.20, 0.01);

    private Position position = null;

    private boolean hasCoral = false;

    private static final double ELEVATOR_DISTANCE_PER_ROTATION = 1.426; // inches
    private static final double ELEVATOR_VELOCITY = 6.0; // inches/second

    private static final double END_EFFECTOR_VELOCITY = Math.PI / 2; // radians/second

    private static final double ALGAE_EXTRACTION_ANGLE = 35.0; // degrees

    private static final double CORAL_INTAKE_POSITION = 0.75;

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

    public boolean hasCoral() {
        return hasCoral;
    }

    public void setElevatorSpeed(double speed) {
        speed *= 0.5;

        SmartDashboard.putNumber("elevator-speed", speed);

        elevatorSparkMax.set(speed);
    }

    public void setEndEffectorSpeed(double speed) {
        SmartDashboard.putNumber("end-effector-speed", speed);

        endEffectorSparkMax.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        speed *= -0.5;

        SmartDashboard.putNumber("intake-speed", speed);

        intakeSparkMax.set(speed);
    }

    public void setPosition(Position position) {
        this.position = position;

        var elevatorPosition = position.elevatorHeight / ELEVATOR_DISTANCE_PER_ROTATION;

        SmartDashboard.putNumber("elevator-height", position.elevatorHeight);

        elevatorSparkMax.getClosedLoopController().setReference(elevatorPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            elevatorFeedForward.calculate(Units.InchesPerSecond.of(ELEVATOR_VELOCITY).in(Units.MetersPerSecond)));

        SmartDashboard.putNumber("end-effector-angle", position.endEffectorAngle);

        var endEffectorPosition = Math.toRadians(position.endEffectorAngle - 90);

        endEffectorSparkMax.getClosedLoopController().setReference(endEffectorPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            endEffectorFeedForward.calculate(endEffectorPosition, END_EFFECTOR_VELOCITY));
    }

    public void extractAlgae(double time) {
        if (position == null) {
            return;
        }

        var endEffectorPosition = Math.toRadians(position.endEffectorAngle - 90 - ALGAE_EXTRACTION_ANGLE);
        var endEffectorVelocity = Units.DegreesPerSecond.of(ALGAE_EXTRACTION_ANGLE / time).in(Units.RadiansPerSecond);

        endEffectorSparkMax.getClosedLoopController().setReference(endEffectorPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            endEffectorFeedForward.calculate(endEffectorPosition, endEffectorVelocity));
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
        SmartDashboard.putNumber("elevator-position", elevatorSparkMax.getEncoder().getPosition());

        SmartDashboard.putBoolean("has-coral", hasCoral);
    }
}
