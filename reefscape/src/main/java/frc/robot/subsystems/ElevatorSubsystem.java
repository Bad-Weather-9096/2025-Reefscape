package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    public enum Position {
        TRANSPORT(32.0, 90.0),
        ALGAE_RELEASE(20.0, 215.0),
        CORAL_INTAKE(24.0, 135.0),
        LOWER_ALGAE_INTAKE(32.0, 180.0),
        UPPER_ALGAE_INTAKE(48.0, 180.0),
        LOWER_CORAL_RELEASE(40.0, 215.0),
        UPPER_CORAL_RELEASE(56.0, 215.0);

        private final double elevatorExtension; // inches
        private final double endEffectorAngle; // degrees

        Position(double elevatorExtension, double endEffectorAngle) {
            this.elevatorExtension = elevatorExtension;
            this.endEffectorAngle = endEffectorAngle;
        }
    }

    private SparkMax elevatorSparkMax;
    private SparkMax endEffectorSparkMax;
    private SparkMax intakeSparkMax;

    private Position position = null;

    private static final int ELEVATOR_CAN_ID = 9;
    private static final int END_EFFECTOR_CAN_ID = 10;
    private static final int INTAKE_CAN_ID = 11;

    private static final double ELEVATOR_DISTANCE_PER_ROTATION = 4.0; // inches

    private static final double ELEVATOR_SPEED = 0.05; // percent
    private static final double MAXIMUM_ELEVATOR_EXTENSION = 72.0; // inches

    private static final double END_EFFECTOR_SPEED = 0.05; // percent
    private static final double MAXIMUM_END_EFFECTOR_ROTATION = 225.0; // degrees

    public ElevatorSubsystem() {
        elevatorSparkMax = new SparkMax(ELEVATOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var elevatorConfig = new SparkMaxConfig();

        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        elevatorSparkMax.configure(elevatorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        elevatorSparkMax.getEncoder().setPosition(0.0);

        endEffectorSparkMax = new SparkMax(END_EFFECTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var endEffectorConfig = new SparkMaxConfig();

        endEffectorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        endEffectorSparkMax.configure(endEffectorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        endEffectorSparkMax.getEncoder().setPosition(0.0);

        intakeSparkMax = new SparkMax(INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(30);

        intakeSparkMax.configure(endEffectorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    public void setElevatorSpeed(double speed) {
        speed *= -0.5;

        SmartDashboard.putNumber("elevator-speed", speed);

        elevatorSparkMax.set(speed);
    }

    public double getElevatorExtension() {
        var position = elevatorSparkMax.getEncoder().getPosition();

        return position * ELEVATOR_DISTANCE_PER_ROTATION;
    }

    public void setIntakeSpeed(double speed) {
        speed *= 0.5;

        SmartDashboard.putNumber("intake-speed", speed);

        intakeSparkMax.set(speed);
    }

    public double getEndEffectorAngle() {
        var position = endEffectorSparkMax.getEncoder().getPosition();

        return position * 360.0;
    }

    public void adjustPosition(Position position) {
        this.position = position;
    }

    @Override
    public void periodic() {
        // TODO
    }
}
