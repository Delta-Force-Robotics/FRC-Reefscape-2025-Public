package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final SparkMax leftIndexerMotor = new SparkMax(IndexerConstants.kLeftIndexerMotorID, MotorType.kBrushless);
    private final SparkMax rightIndexerMotor = new SparkMax(IndexerConstants.kRightIndexerMotorID, MotorType.kBrushless);

    private SparkMaxConfig leftIndexerMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig rightIndexerMotorConfig = new SparkMaxConfig();

    public DigitalInput indexerBreakBeam = new DigitalInput(IndexerConstants.kBreakBeamID);

    public IndexerSubsystem() {
        configureLeftIndexerMotor();
        configureRightIndexerMotor();
    }

    public Command runIndexer(double power) {
        return this.runOnce(() -> leftIndexerMotor.set(power));
    }

    public BooleanSupplier isBreakBeamTripped() {
        return () -> !indexerBreakBeam.get() && (leftIndexerMotor.getAppliedOutput() != 0);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexer Break Beam:", !indexerBreakBeam.get());
        SmartDashboard.updateValues();
    }

    public void configureLeftIndexerMotor() {
        leftIndexerMotorConfig
         .inverted(true)
         .smartCurrentLimit(IndexerConstants.kSmartCurrentLimit)
         .idleMode(IdleMode.kCoast);

        leftIndexerMotor.configure(leftIndexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void configureRightIndexerMotor() {
        rightIndexerMotorConfig
         .smartCurrentLimit(IndexerConstants.kSmartCurrentLimit)
         .idleMode(IdleMode.kCoast)
         .follow(leftIndexerMotor, true);

        rightIndexerMotor.configure(rightIndexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}