package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Aligns the robot to the left or right reef branch using the robot pose relative to the AprilTag
 * that is visible when the command starts.
 *
 * <p>Ends once the robot has held the aligned pose for {@link AutoAlignConstants#kPoseValidationTime},
 * or when the tag has not been seen for {@link AutoAlignConstants#kLostTagTimeout}.
 */
public class AlignToReefTagRelative extends Command {
  private static final String kLimelight = AutoAlignConstants.kLimelightName;

  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric driveRequest =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final PIDController xController = new PIDController(AutoAlignConstants.kXP, 0, 0);
  private final PIDController yController = new PIDController(AutoAlignConstants.kYP, 0, 0);
  private final PIDController rotController = new PIDController(AutoAlignConstants.kRotP, 0, 0);

  private final Timer lostTagTimer = new Timer();
  private final Timer alignedTimer = new Timer();

  private double tagId = -1;

  /**
   * @param rightBranch true to align to the right reef branch, false for the left one
   * @param drivetrain the swerve drivetrain
   */
  public AlignToReefTagRelative(boolean rightBranch, CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    xController.setSetpoint(rightBranch ? AutoAlignConstants.kRightXSetpoint : AutoAlignConstants.kLeftXSetpoint);
    yController.setSetpoint(rightBranch ? AutoAlignConstants.kRightYSetpoint : AutoAlignConstants.kLeftYSetpoint);
    rotController.setSetpoint(rightBranch ? AutoAlignConstants.kRightRotSetpoint : AutoAlignConstants.kLeftRotSetpoint);

    xController.setTolerance(AutoAlignConstants.kXTolerance);
    yController.setTolerance(AutoAlignConstants.kYTolerance);
    rotController.setTolerance(AutoAlignConstants.kRotTolerance);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotController.reset();
    lostTagTimer.restart();
    alignedTimer.restart();

    tagId = LimelightHelpers.getFiducialID(kLimelight);
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(kLimelight) || LimelightHelpers.getFiducialID(kLimelight) != tagId) {
      // Without a tag we cannot tell whether we are aligned, so never finish on the aligned timer.
      alignedTimer.reset();
      stop();
      return;
    }

    lostTagTimer.reset();

    // [x, y, z, roll, pitch, yaw] of the robot in the tag's coordinate frame
    double[] pose = LimelightHelpers.getBotPose_TargetSpace(kLimelight);
    double xSpeed = -xController.calculate(pose[2]);
    double ySpeed = -yController.calculate(pose[0]);
    double rotSpeed = -rotController.calculate(pose[4]) * AutoAlignConstants.kRotOutputScale;

    drivetrain.setControl(
        driveRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotSpeed));

    if (!xController.atSetpoint() || !yController.atSetpoint() || !rotController.atSetpoint()) {
      alignedTimer.reset();
    }

    SmartDashboard.putNumber("AutoAlign/Aligned Time", alignedTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    stop();
  }

  @Override
  public boolean isFinished() {
    return lostTagTimer.hasElapsed(AutoAlignConstants.kLostTagTimeout)
        || alignedTimer.hasElapsed(AutoAlignConstants.kPoseValidationTime);
  }

  private void stop() {
    drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }
}
