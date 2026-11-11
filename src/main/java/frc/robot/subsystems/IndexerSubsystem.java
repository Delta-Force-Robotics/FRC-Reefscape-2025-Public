package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

/** Indexer that feeds coral from the intake to the gripper. The right motor follows the left. */
public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax leftMotor = new SparkMax(IndexerConstants.kLeftIndexerMotorID, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(IndexerConstants.kRightIndexerMotorID, MotorType.kBrushless);
  private final DigitalInput breakBeam = new DigitalInput(IndexerConstants.kBreakBeamID);

  public IndexerSubsystem() {
    configureMotors();
  }

  public Command runIndexer(double power) {
    return runOnce(() -> leftMotor.set(power));
  }

  public boolean isBreakBeamBroken() {
    return !breakBeam.get();
  }

  /** True when a coral reaches the end of the indexer while it is running. */
  public boolean hasCoral() {
    return isBreakBeamBroken() && leftMotor.getAppliedOutput() != 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Indexer/Break Beam", isBreakBeamBroken());
  }

  private void configureMotors() {
    var leftConfig = new SparkMaxConfig();
    leftConfig
        .inverted(true)
        .smartCurrentLimit(IndexerConstants.kSmartCurrentLimit)
        .idleMode(IdleMode.kCoast);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var rightConfig = new SparkMaxConfig();
    rightConfig
        .smartCurrentLimit(IndexerConstants.kSmartCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .follow(leftMotor, true);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
