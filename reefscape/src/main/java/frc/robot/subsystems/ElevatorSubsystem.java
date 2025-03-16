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

    public ElevatorSubsystem() {
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
    }

    public void setElevatorPosition(ElevatorPosition elevatorPosition) {
        elevatorSparkMax.getClosedLoopController().setReference(elevatorPosition.getRotations(), SparkBase.ControlType.kPosition);
    }

    public void setElevatorSpeed(double speed) {
        elevatorSparkMax.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator-position", elevatorSparkMax.getEncoder().getPosition());
    }
}
