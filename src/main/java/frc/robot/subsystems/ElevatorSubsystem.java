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

    private SparkMax intakeSparkMax1 = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
    private SparkMax intakeSparkMax2 = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);

    private static final double CORAL_LENGTH = 11.875; // inches
    private static final double INTAKE_WHEEL_DIAMETER = 2.25; // inches

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

        intakeSparkMax1.configure(intakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        intakeSparkMax1.getEncoder().setPosition(0);

        intakeSparkMax2.configure(intakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        intakeSparkMax2.getEncoder().setPosition(0);
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
        var rotations = CORAL_LENGTH / (INTAKE_WHEEL_DIAMETER * Math.PI) * multiplier;

        intakeSparkMax1.getClosedLoopController().setReference(intakeSparkMax1.getEncoder().getPosition() + rotations,
            SparkBase.ControlType.kPosition);

        intakeSparkMax2.getClosedLoopController().setReference(intakeSparkMax2.getEncoder().getPosition() - rotations,
            SparkBase.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator-position", elevatorSparkMax.getEncoder().getPosition());

        SmartDashboard.putNumber("intake1-position", intakeSparkMax1.getEncoder().getPosition());
        SmartDashboard.putNumber("intake2-position", intakeSparkMax2.getEncoder().getPosition());
    }
}
