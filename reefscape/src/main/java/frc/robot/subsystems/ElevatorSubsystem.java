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

    private boolean hasCoral = false;

    private static final double INITIAL_END_EFFECTOR_ANGLE = -42.5; // degrees

    private static final double CORAL_INTAKE_POSITION = 0.75; // percent

    public ElevatorSubsystem() {
        // Elevator
        var elevatorConfig = new SparkMaxConfig();

        elevatorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        elevatorConfig.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .p(0.20)
            .i(0)
            .d(0)
            .outputRange(-1, 1);

        elevatorSparkMax.configure(elevatorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        elevatorSparkMax.getEncoder().setPosition(0);

        // End effector
        var endEffectorConfig = new SparkMaxConfig();

        endEffectorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        endEffectorConfig.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .p(0.18)
            .i(0)
            .d(0)
            .outputRange(-4, 4);

        endEffectorSparkMax.configure(endEffectorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        endEffectorSparkMax.getEncoder().setPosition(0);

        // Intake
        var intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(30);

        intakeSparkMax.configure(intakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public void setElevatorPower(double power) {
        elevatorSparkMax.set(power);
    }

    public void setElevatorPosition(double position) {
        elevatorSparkMax.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
    }

    public void setEndEffectorPosition(double position) {
        endEffectorSparkMax.getClosedLoopController().setReference(position * 4, SparkBase.ControlType.kPosition);
    }

    public void setIntakeSpeed(double speed) {
        speed *= -0.5;

        SmartDashboard.putNumber("intake-speed", speed);

        intakeSparkMax.set(speed);
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
        SmartDashboard.putBoolean("has-coral", hasCoral);
    }
}
