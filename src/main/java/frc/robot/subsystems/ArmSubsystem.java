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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase { 
    private TalonFX armMotor = new TalonFX(ArmConstants.kArmMotorID, Constants.kCanBus);
    private CANcoder canCoder = new CANcoder(ArmConstants.kCanCoderID, Constants.kCanBus);

    private final MotionMagicDutyCycle motionMagicDutyCycle 
    = new MotionMagicDutyCycle(0).withEnableFOC(true).withSlot(0);

    public ScoringTarget scoringTarget = ScoringTarget.L4;
    public AlgaePickUp algaePickUp = AlgaePickUp.FLOOR;
    public int scoringIncrement = 5;
    public int algaeIncrement = 1;

    public ArmSubsystem() {
        configureArmPivot();
    }

    public Command moveArm(double position) {
        return this.runOnce(
            () -> armMotor.setControl(motionMagicDutyCycle.withPosition(position)));
    }

    public Command goToTarget() {
        return this.runOnce(
            () -> armMotor.setControl(motionMagicDutyCycle.withPosition(scoringTarget.getTargetRotations())));
    }

    public void scoreRelease() {
        if (getScoringTarget() == ScoringTarget.PROCESSOR || getScoringTarget() == ScoringTarget.BARGE) {
            armMotor.setControl(motionMagicDutyCycle.withPosition(scoringTarget.getTargetRotations()));
        } else if(getScoringTarget() == ScoringTarget.L1) {
            armMotor.setControl(motionMagicDutyCycle.withPosition(scoringTarget.getTargetRotations() - 0.045));
        } else {
            armMotor.setControl(motionMagicDutyCycle.withPosition(scoringTarget.getTargetRotations() - 0.07));
        }
    }

    public Command pickUpAlgae() {
        return this.runOnce(
            () -> armMotor.setControl(motionMagicDutyCycle.withPosition(algaePickUp.getTargetRotations())));
    }

    public ScoringTarget getScoringTarget() {
        return this.scoringTarget;
    }

    public AlgaePickUp getAlgaePickUp() {
        return this.algaePickUp;
    }

    public double getArmPosition() {
        return armMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position: ", armMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm Rotations", scoringTarget.getTargetRotations());
        SmartDashboard.updateValues();
    }

    public enum ScoringTarget {
        L4(ArmConstants.kL4Pos),
        L3(ArmConstants.kL3Pos),
        L2(ArmConstants.kL2Pos),
        L1(ArmConstants.kL1Pos),
        BARGE(ArmConstants.kBargePos),
        PROCESSOR(ArmConstants.kProcessorPos);

        private final double targetRotations;

        ScoringTarget(double targetRotations) {
            this.targetRotations = targetRotations;
        }

        public double getTargetRotations() {
            return targetRotations;
        }
    }

    public enum AlgaePickUp {
        ALGAEL2(ArmConstants.kAlgaeL2Pos),
        ALGAEL3(ArmConstants.kAlgaeL3Pos),
        FLOOR(ArmConstants.kAlgaeFloorPos);

        private final double targetRotations;

        AlgaePickUp(double targetRotations) {
            this.targetRotations = targetRotations;
        }

        public double getTargetRotations() {
            return targetRotations;
        }
    }

    public void increaseScoringTarget() {
        if (scoringIncrement < 6) {scoringIncrement++;}
        if (scoringIncrement == 1) {scoringTarget = ScoringTarget.PROCESSOR;}
        else if (scoringIncrement == 2) {scoringTarget = ScoringTarget.L1;}
        else if (scoringIncrement == 3) {scoringTarget = ScoringTarget.L2;}
        else if (scoringIncrement == 4) {scoringTarget = ScoringTarget.L3;}
        else if (scoringIncrement == 5) {scoringTarget = ScoringTarget.L4;}
        else if (scoringIncrement == 6) {scoringTarget = ScoringTarget.BARGE;}
    }

    public void decreaseScoringTarget() {
        if (scoringIncrement > 1) {scoringIncrement--;}
        if (scoringIncrement == 1) {scoringTarget = ScoringTarget.PROCESSOR;}
        else if (scoringIncrement == 2) {scoringTarget = ScoringTarget.L1;}
        else if (scoringIncrement == 3) {scoringTarget = ScoringTarget.L2;}
        else if (scoringIncrement == 4) {scoringTarget = ScoringTarget.L3;}
        else if (scoringIncrement == 5) {scoringTarget = ScoringTarget.L4;}
        else if (scoringIncrement == 6) {scoringTarget = ScoringTarget.BARGE;}
    }

    public void increaseAlgaePickUp() {
        if (algaeIncrement < 3) {algaeIncrement++;}
        if (algaeIncrement == 1) {algaePickUp = AlgaePickUp.FLOOR;}
        else if (algaeIncrement == 2) {algaePickUp = AlgaePickUp.ALGAEL2;}
        else if (algaeIncrement == 3) {algaePickUp = AlgaePickUp.ALGAEL3;}
    }

    public void decreaseAlgaePickUp() {
        if (algaeIncrement > 1) {algaeIncrement--;}
        if (algaeIncrement == 1) {algaePickUp = AlgaePickUp.FLOOR;}
        else if (algaeIncrement == 2) {algaePickUp = AlgaePickUp.ALGAEL2;}
        else if (algaeIncrement == 3) {algaePickUp = AlgaePickUp.ALGAEL3;}
    }

    private void configureArmPivot() {
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = ArmConstants.kP;
        slot0Configs.kI = ArmConstants.kI;
        slot0Configs.kD = ArmConstants.kD;
        slot0Configs.kG = ArmConstants.kG;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.kCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.kAcceleration;
        
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ArmConstants.kStatorCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ArmConstants.kSupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfigs.Feedback.SensorToMechanismRatio = ArmConstants.kSensorToMechanismRatio;
        talonFXConfigs.Feedback.RotorToSensorRatio = ArmConstants.kRotorToSensorRatio;

        armMotor.getConfigurator().apply(talonFXConfigs);
    }
}