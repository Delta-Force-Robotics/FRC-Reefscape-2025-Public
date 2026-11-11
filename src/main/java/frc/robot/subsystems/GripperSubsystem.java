package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
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
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase { 
    private final SparkMax gripperMotor = new SparkMax(GripperConstants.kGripperMotorID, MotorType.kBrushless); 
    private SparkMaxConfig gripperConfig = new SparkMaxConfig();

    public DigitalInput gripperBreakBeam = new DigitalInput(GripperConstants.kBreakBeamID);

    public boolean intakingAlgae = false;

    public GripperSubsystem() {
        configureGripper();
    }

    public Command runGripper(double power) {
        return this.runOnce(() -> gripperMotor.set(power));
    }

    public Command runPassiveGripper(double power) {
        return this.runOnce(() -> gripperMotor.set(power));
    }
    
    public BooleanSupplier isBreakBeamTripped() {
        return () -> !gripperBreakBeam.get() && intakingAlgae;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Gripper Break Beam:", !gripperBreakBeam.get());
        SmartDashboard.putBoolean("Intaking Algae: ", intakingAlgae);
        SmartDashboard.updateValues();
    }

    public void configureGripper() {
        gripperConfig
         .inverted(true)
         .smartCurrentLimit(GripperConstants.kSmartCurrentLimit)
         .idleMode(IdleMode.kBrake);

        gripperMotor.configure(gripperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}