package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private double tagID = -1;
  private double xSetPoint, ySetPoint, rotSetPoint;
  private double xTolerance, yTolerance, rotTolerance;
  private CommandSwerveDrivetrain drivebase;
  private SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(AutoAlignConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(AutoAlignConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(AutoAlignConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (isRightScore) {
      xSetPoint = AutoAlignConstants.RIGHT_X_SETPOINT_REEF_ALIGNMENT;
      xTolerance = AutoAlignConstants.RIGHT_X_TOLERANCE_REEF_ALIGNMENT;

      ySetPoint = AutoAlignConstants.RIGHT_Y_SETPOINT_REEF_ALIGNMENT;
      yTolerance = AutoAlignConstants.RIGHT_Y_TOLERANCE_REEF_ALIGNMENT;

      rotSetPoint = AutoAlignConstants.RIGHT_ROT_SETPOINT_REEF_ALIGNMENT;
      rotTolerance = AutoAlignConstants.ROT_TOLERANCE_REEF_ALIGNMENT;
      } else {
      xSetPoint = AutoAlignConstants.LEFT_X_SETPOINT_REEF_ALIGNMENT;
      xTolerance = AutoAlignConstants.LEFT_X_TOLERANCE_REEF_ALIGNMENT;
  
      ySetPoint = AutoAlignConstants.LEFT_Y_SETPOINT_REEF_ALIGNMENT;
      yTolerance = AutoAlignConstants.LEFT_Y_TOLERANCE_REEF_ALIGNMENT;
  
      rotSetPoint = AutoAlignConstants.LEFT_ROT_SETPOINT_REEF_ALIGNMENT;
      rotTolerance = AutoAlignConstants.ROT_TOLERANCE_REEF_ALIGNMENT;
      }
  
      xController.setSetpoint(xSetPoint);
      xController.setTolerance(xTolerance);
  
      yController.setSetpoint(ySetPoint);
      yController.setTolerance(yTolerance);

      rotController.setSetpoint(rotSetPoint);
      rotController.setTolerance(rotTolerance);

    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);
      SmartDashboard.putNumber("y", postions[0]);
      SmartDashboard.putNumber("rot", postions[4]);

      double xSpeed = -xController.calculate(postions[2]);
      SmartDashboard.putNumber("xSpeed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      drivebase.setControl(
        drive.withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(rotValue * 0.2)
      );

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.setControl(
        drive.withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
      );
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setControl(
      drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0)
    );
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(AutoAlignConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(AutoAlignConstants.POSE_VALIDATION_TIME);
  }
}