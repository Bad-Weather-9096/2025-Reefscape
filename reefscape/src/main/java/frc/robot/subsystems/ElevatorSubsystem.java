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

    private boolean hasCoral = false;
    private boolean hasAlgae = false;

    private static final int ELEVATOR_CAN_ID = 9;
    private static final int END_EFFECTOR_CAN_ID = 10;

    private static final int ALGAE_INTAKE_CAN_ID = 11;

    private static final double CAMERA_OFFSET = 4.25; // inches

    // TODO Elevator constants
    private static final int TICKS_PER_INCH = 24;

    private static final double ELEVATOR_SPEED = 0.2; // percent
    private static final double MAXIMUM_EXTENSION = 24.0; // inches

    // TODO End effector constants
    private static final int TICKS_PER_DEGREE = 24;

    private static final double END_EFFECTOR_SPEED = 0.1; // percent
    private static final double MAXIMUM_ROTATION = 270.0; // degrees

    // TODO Algae intake constants
    private static final double ALGAE_INTAKE_SPEED = 0.1; // percent

    // TODO Coral intake constants
    private static final double CORAL_INTAKE_POSITION = 0.5;

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

    public double getCameraHeight() {
        return getElevatorExtension() + CAMERA_OFFSET;
    }

    public double getElevatorExtension() {
        return elevatorController.getEncoder().getPosition() / TICKS_PER_INCH;
    }

    public void raiseElevator() {
        elevatorController.set(getElevatorExtension() < MAXIMUM_EXTENSION ? ELEVATOR_SPEED : 0.0);
    }

    public void lowerElevator() {
        elevatorController.set(getElevatorExtension() > 0 ? -ELEVATOR_SPEED : 0.0);
    }

    public void stopElevator() {
        elevatorController.set(0.0);
    }

    public double getEndEffectorRotation() {
        return endEffectorController.getEncoder().getPosition() / TICKS_PER_DEGREE;
    }

    public void raiseEndEffector() {
        endEffectorController.set(getEndEffectorRotation() > 0 ? -END_EFFECTOR_SPEED : 0.0);
    }

    public void lowerEndEffector() {
        endEffectorController.set(getEndEffectorRotation() < MAXIMUM_ROTATION ? END_EFFECTOR_SPEED : 0.0);
    }

    public void stopEndEffector() {
        endEffectorController.set(0.0);
    }

    public void receiveCoral() {
        coralIntakeServo.set(CORAL_INTAKE_POSITION);

        hasCoral = true;
    }

    public void releaseCoral() {
        coralIntakeServo.set(-CORAL_INTAKE_POSITION);

        hasCoral = false;
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public void receiveAlgae() {
        algaeIntakeController.set(ALGAE_INTAKE_SPEED);

        hasAlgae = true;
    }

    public void releaseAlgae() {
        // TODO Reverse briefly
        algaeIntakeController.set(0.0);

        hasAlgae = false;
    }

    public boolean hasAlgae() {
        return hasAlgae;
    }

    public void adjustPosition(ElevatorPosition elevatorLevel) {
        // TODO Ignore if already adjusting position or position height is within tolerance
        // TODO Show height in dashboard so operator can see it

        // TODO IMPORTANT This method also needs to adjust the end effector angle
    }
}
