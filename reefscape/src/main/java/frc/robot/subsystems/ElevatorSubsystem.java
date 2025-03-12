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

    private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(0.22, 0.81, 3.07, 0.10);
    private ArmFeedforward endEffectorFeedForward = new ArmFeedforward(0.22, 0.75, 0.20, 0.01);

    private Position position = null;

    private boolean hasCoral = false;

    // TODO
    private static final double INITIAL_END_EFFECTOR_ANGLE = -42.5; // degrees

    private static final double ELEVATOR_DISTANCE_PER_ROTATION = 1.426; // inches
    private static final double ELEVATOR_VELOCITY = 6.0; // inches/second

    private static final double END_EFFECTOR_VELOCITY = 45.0; // degrees/second

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

        endEffectorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        endEffectorSparkMax.configure(endEffectorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        endEffectorSparkMax.getEncoder().setPosition(INITIAL_END_EFFECTOR_ANGLE / 360);

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
        speed *= 0.25;

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
        var elevatorVelocity = Units.InchesPerSecond.of(ELEVATOR_VELOCITY).in(Units.MetersPerSecond);

        var elevatorFF = elevatorFeedForward.calculate(elevatorVelocity);

        System.out.printf("Elevator position = %.2f, velocity = %.2f m/s, FF = %.2f m/s\n",
            elevatorPosition,
            elevatorVelocity,
            elevatorFF);

        elevatorSparkMax.getClosedLoopController().setReference(elevatorPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            elevatorFF);

        // TODO Rotations must take gear ratio into account
        var endEffectorPosition = position.endEffectorAngle / 360; // rotations
        var endEffectorVelocity = Units.DegreesPerSecond.of(END_EFFECTOR_VELOCITY).in(Units.RadiansPerSecond);

        var endEffectorFF = endEffectorFeedForward.calculate(Math.toRadians(position.endEffectorAngle - 90), endEffectorVelocity);

        System.out.printf("End effector position = %.2f rotations, velocity = %.2f rad/s, FF = %.2f rad/s\n",
            endEffectorPosition,
            endEffectorVelocity,
            endEffectorFF);

        endEffectorSparkMax.getClosedLoopController().setReference(endEffectorPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            endEffectorFF);
    }

    public void extractAlgae(double time) {
        if (position == null) {
            return;
        }

        var elevatorPosition = (position.elevatorHeight + ALGAE_EXTRACTION_HEIGHT) / ELEVATOR_DISTANCE_PER_ROTATION;
        var elevatorVelocity = Units.InchesPerSecond.of(ALGAE_EXTRACTION_HEIGHT / time).in(Units.MetersPerSecond);

        var elevatorFF = elevatorFeedForward.calculate(elevatorVelocity);

        System.out.printf("Elevator position = %.2f, velocity = %.2f m/s, FF = %.2f m/s\n",
            elevatorPosition,
            elevatorVelocity,
            elevatorFF);

        elevatorSparkMax.getClosedLoopController().setReference(elevatorPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            elevatorFF);

        var endEffectorPosition = Math.toRadians(position.endEffectorAngle - ALGAE_EXTRACTION_ANGLE);
        var endEffectorVelocity = Units.DegreesPerSecond.of(ALGAE_EXTRACTION_ANGLE / (time / 4)).in(Units.RadiansPerSecond);

        var endEffectorFF = endEffectorFeedForward.calculate(endEffectorPosition - Math.PI / 2, endEffectorVelocity);

        System.out.printf("End effector position = %.2f, velocity = %.2f rad/s, FF = %.2f rad/s\n",
            endEffectorPosition,
            endEffectorVelocity,
            endEffectorFF);

        endEffectorSparkMax.getClosedLoopController().setReference(endEffectorPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            endEffectorFF);
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
        SmartDashboard.putNumber("end-effector-position", endEffectorSparkMax.getEncoder().getPosition());

        SmartDashboard.putString("position", (position == null) ? "" : position.toString());

        SmartDashboard.putBoolean("has-coral", hasCoral);
    }
}
