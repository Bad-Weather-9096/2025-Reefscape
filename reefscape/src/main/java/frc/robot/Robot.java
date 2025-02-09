package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Robot class.
 */
public class Robot extends TimedRobot {
  private XboxController driveController = new XboxController(0);
  private XboxController elevatorController = new XboxController(1);

  private DriveSubsystem driveSubsystem = new DriveSubsystem();

  private static final double DRIVE_DEADBAND = 0.05;

  private static final String X_SPEED_KEY = "X speed";
  private static final String Y_SPEED_KEY = "Y speed";

  private static final String ROTATION_KEY = "Rotation";

  @Override
  public void teleopPeriodic() {
    if (elevatorController.getXButton()) {
      SmartDashboard.putNumber(X_SPEED_KEY, 0.0);
      SmartDashboard.putNumber(Y_SPEED_KEY, 0.0);

      SmartDashboard.putNumber(ROTATION_KEY, 0.0);

      driveSubsystem.setX();
    } else {
      var xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
      var ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

      SmartDashboard.putNumber(X_SPEED_KEY, xSpeed);
      SmartDashboard.putNumber(Y_SPEED_KEY, ySpeed);

      var rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

      SmartDashboard.putNumber(ROTATION_KEY, rot);

      driveSubsystem.drive(xSpeed, ySpeed, rot, true);
    }
  }
}
