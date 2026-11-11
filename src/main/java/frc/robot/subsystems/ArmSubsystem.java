package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import java.util.function.DoubleSupplier;

/** Coral/algae arm pivot, driven by a Kraken with a fused CANcoder and Motion Magic. */
public class ArmSubsystem extends SubsystemBase {
  private final TalonFX armMotor = new TalonFX(ArmConstants.kArmMotorID, Constants.kCanBus);
  private final CANcoder canCoder = new CANcoder(ArmConstants.kCanCoderID, Constants.kCanBus);

  private final MotionMagicDutyCycle motionMagicRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(true).withSlot(0);

  public ArmSubsystem() {
    configureArmMotor();
  }

  /** Moves the arm to a fixed position. */
  public Command moveArm(double position) {
    return moveArm(() -> position);
  }

  /** Moves the arm to a position evaluated when the command runs. */
  public Command moveArm(DoubleSupplier position) {
    return runOnce(() -> armMotor.setControl(motionMagicRequest.withPosition(position.getAsDouble())));
  }

  public double getPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Position", getPosition());
  }

  private void configureArmMotor() {
    var config = new TalonFXConfiguration();

    config.Slot0.kP = ArmConstants.kP;
    config.Slot0.kI = ArmConstants.kI;
    config.Slot0.kD = ArmConstants.kD;
    config.Slot0.kG = ArmConstants.kG;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.kCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = ArmConstants.kAcceleration;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimit = ArmConstants.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ArmConstants.kSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = ArmConstants.kSensorToMechanismRatio;
    config.Feedback.RotorToSensorRatio = ArmConstants.kRotorToSensorRatio;

    armMotor.getConfigurator().apply(config);
  }
}
