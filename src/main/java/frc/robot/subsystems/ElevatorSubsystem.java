package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax elevatorSparkMax = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
    private SparkMax intakeSparkMax = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);

    private static final double CORAL_LENGTH = 11.875; // inches
    private static final double INTAKE_WHEEL_DIAMETER = 2.25; // inches
    private static final double INTAKE_GEAR_RATIO = 1.0;

    public ElevatorSubsystem() {
        var elevatorConfig = new SparkMaxConfig();

        elevatorConfig.smartCurrentLimit(30);

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

        var intakeConfig = new SparkMaxConfig();

        intakeConfig.smartCurrentLimit(20);

        intakeConfig.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1);

        intakeSparkMax.configure(intakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);
    }

    public void setElevatorPosition(ElevatorPosition elevatorPosition) {
        elevatorSparkMax.getClosedLoopController().setReference(elevatorPosition.getRotations(), SparkBase.ControlType.kPosition);
    }

    public void setElevatorSpeed(double speed) {
        elevatorSparkMax.set(speed);
    }

    public void receiveCoral() {
        moveCoral(1.0);
    }

    public void releaseCoral() {
        moveCoral(1.25);
    }

    private void moveCoral(double multiplier) {
        var position = intakeSparkMax.getEncoder().getPosition();

        var rotations = CORAL_LENGTH / (INTAKE_WHEEL_DIAMETER * Math.PI) * multiplier;

        intakeSparkMax.getClosedLoopController().setReference(position + rotations * INTAKE_GEAR_RATIO, SparkBase.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator-position", elevatorSparkMax.getEncoder().getPosition());
        SmartDashboard.putNumber("intake-position", intakeSparkMax.getEncoder().getPosition());
    }
}
