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

  @Override
  public void teleopPeriodic() {
    if (elevatorController.getXButton()) {
      driveSubsystem.setX();
    } else {
      var xSpeed = -MathUtil.applyDeadband(driveController.getLeftY(), DRIVE_DEADBAND);
      var ySpeed = -MathUtil.applyDeadband(driveController.getLeftX(), DRIVE_DEADBAND);

      SmartDashboard.putNumber("X speed", xSpeed);
      SmartDashboard.putNumber("Y speed", ySpeed);

      var rot = -MathUtil.applyDeadband(driveController.getRightX(), DRIVE_DEADBAND);

      SmartDashboard.putNumber("Rotation", rot);

      driveSubsystem.drive(xSpeed, ySpeed, rot, true);
    }
  }

  @Override
  public void testPeriodic() {
    if (elevatorController.getYButton()) {
      driveSubsystem.resetEncoders();
    }
  }
}
