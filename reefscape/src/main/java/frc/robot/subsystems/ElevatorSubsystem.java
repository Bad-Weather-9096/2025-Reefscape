package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax elevatorController;
    private SparkMax armController;

    private Servo endEffectorServo;

    private static final int ELEVATOR_CAN_ID = 9;
    private static final int ARM_CAN_ID = 10;

    // TODO
    private static final double MAXIMUM_HEIGHT = 24.0; // inches

    // TODO
    private static final double ELEVATOR_SPEED = 0.1; // percent

    // TODO
    private static final double END_EFFECTOR_UP_ANGLE = 90;
    private static final double END_EFFECTOR_DOWN_ANGLE = -90;

    public ElevatorSubsystem() {
        elevatorController = new SparkMax(ELEVATOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var elevatorConfig = new SparkMaxConfig();

        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        elevatorController.configure(elevatorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        elevatorController.getEncoder().setPosition(0);

        armController = new SparkMax(ARM_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var armConfig = new SparkMaxConfig();

        armConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        armController.configure(armConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        armController.getEncoder().setPosition(0);

        endEffectorServo = new Servo(0);
    }

    public double getHeight() {
        // TODO Convert ticks to inches
        return 0.0;
    }

    public double getCameraHeight() {
        // TODO Return elevator height minus camera offset
        return 9.25;
    }

    public void raiseElevator() {
        elevatorController.set(getHeight() < MAXIMUM_HEIGHT ? ELEVATOR_SPEED : 0.0);
    }

    public void lowerElevator() {
        elevatorController.set(getHeight() > 0 ? -ELEVATOR_SPEED : 0.0);
    }

    public void raiseEndEffector() {
        endEffectorServo.set(END_EFFECTOR_UP_ANGLE);
    }

    public void lowerEndEffector() {
        endEffectorServo.set(END_EFFECTOR_DOWN_ANGLE);
    }
}
