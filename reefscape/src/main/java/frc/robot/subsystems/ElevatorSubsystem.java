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
    private SparkMax endEffectorController;

    private SparkMax algaeIntakeController;

    private Servo coralIntakeServo;

    private static final int ELEVATOR_CAN_ID = 9;
    private static final int END_EFFECTOR_CAN_ID = 10;

    private static final int ALGAE_INTAKE_CAN_ID = 11;

    // TODO
    private static final int TICKS_PER_INCH = 24;

    // TODO
    private static final double MAXIMUM_EXTENSION = 24.0; // inches
    private static final double CAMERA_OFFSET = 4.25; // inches

    // TODO
    private static final double ELEVATOR_SPEED = 0.1; // percent

    public ElevatorSubsystem() {
        elevatorController = new SparkMax(ELEVATOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var elevatorConfig = new SparkMaxConfig();

        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        elevatorController.configure(elevatorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        elevatorController.getEncoder().setPosition(0);

        endEffectorController = new SparkMax(END_EFFECTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var endEffectorConfig = new SparkMaxConfig();

        endEffectorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        endEffectorController.configure(endEffectorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        endEffectorController.getEncoder().setPosition(0);

        algaeIntakeController = new SparkMax(ALGAE_INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        var algaeIntakeConfig = new SparkMaxConfig();

        algaeIntakeConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        algaeIntakeController.configure(algaeIntakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

        coralIntakeServo = new Servo(0);
    }

    private double getExtension() {
        return elevatorController.getEncoder().getPosition() / TICKS_PER_INCH;
    }

    public double getCameraHeight() {
        return getExtension() + CAMERA_OFFSET;
    }

    public void raiseElevator() {
        elevatorController.set(getExtension() < MAXIMUM_EXTENSION ? ELEVATOR_SPEED : 0.0);
    }

    public void lowerElevator() {
        elevatorController.set(getExtension() > 0 ? -ELEVATOR_SPEED : 0.0);
    }

    public void stopElevator() {
        elevatorController.set(0.0);
    }

    public void raiseEndEffector() {
        // TODO
    }

    public void lowerEndEffector() {
        // TODO
    }

    public void stopEndEffector() {
        // TODO
    }

    public void moveLeft() {
        // TODO Ignore if already moving left or right
    }

    public void moveRight() {
        // TODO Ignore if already moving left or right
    }

    public void selectBranchLevel(int level) {
        // TODO Ignore if branch level is currently being set
        // TODO Left/right = coral, center = algae
    }

    public void receiveCoral() {
        // TODO Set servo position
    }

    public void releaseCoral() {
        // TODO Set servo position
    }

    public void receiveAlgae() {
        // TODO Reverse low speed
    }

    public void releaseAlgae() {
        // TODO Forward low speed
    }
}
