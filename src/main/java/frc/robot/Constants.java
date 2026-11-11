// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String kCanBus = "ctre";
  public static final double X_REEF_ALIGNMENT_P = 0;

  public final class ElevatorConstants {
    public static final int kFrontMotorID = 15;
    public static final int kBackMotorID = 14;
    
    // Motion Magic parameters
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0.01;
    public static final double kG = 0;
    public static final double kCruiseVelocity = 400;
    public static final double kAcceleration = 400;
    public static final double kStatorCurrentLimit = 120;
    public static final double kSupplyCurrentLimit = 70;

    // Scoring and stowed positions
    public static final double kHomePos = 0.1;
    public static final double kWaitForCoralPos = 3;
    public static final double kInterPos = 5;
    public static final double kL1Pos = -5.870;
    public static final double kL2Pos = -4.83;
    public static final double kL3Pos = 4.4;
    public static final double kL4Pos = 19.45;
    public static final double kBargePos = 25.52;
    public static final double kProcessorPos = -5.977;

    // Algae pickup positions
    public static final double kAlgaeFloorPos = -5.977;
    public static final double kAlgaeL2Pos = 2.66;
    public static final double kAlgaeL3Pos = 13.8;
  }

  public final class ArmConstants {
    public static final int kArmMotorID = 16;
    public static final int kCanCoderID = 17;

    // Motion Magic parameters
    public static final double kP = 6.3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kCruiseVelocity = 270;
    public static final double kAcceleration = 120;
    public static final double kStatorCurrentLimit = 120;
    public static final double kSupplyCurrentLimit = 70;
    public static final double kSensorToMechanismRatio = 3;
    public static final double kRotorToSensorRatio = 25;

    // Scoring and home positions
    public static final double kHomePos = -0.0012;
    public static final double kProcessorPos = 0.2357;
    public static final double kL1Pos = 0.303;
    public static final double kL2Pos = 0.358;
    public static final double kL3Pos = 0.376;
    public static final double kL4Pos = 0.4;
    public static final double kBargePos = 0.445;

    // Algae pickup positions
    public static final double kAlgaeFloorPos = 0.1857;
    public static final double kAlgaeL2Pos = 0.255;
    public static final double kAlgaeL3Pos = 0.255;
  }

  public final class GripperConstants {
    public static final int kGripperMotorID = 8;
    public static final int kBreakBeamID = 0;

    public static final int kSmartCurrentLimit = 40;

    // Speeds
    public static final double kCoralPassivePow = 0;
    public static final double kAlgaePassivePow = 0.15;
    public static final double kIntakePow = 1;
    public static final double kOuttakePow = -1;
    public static final double kStopPow = 0;

  }

  public final class IntakeConstants {
    public static final int kIntakeMotorID = 4;
    public static final int kPivotMotorID = 3;

    // PID parameters
    public static final double kP = 0.9;
    public static final double kI = 0;
    public static final double kD = 0.4;
    public static final int kSmartCurrentLimitPivot = 40;
    public static final int kSmartCurrentLimitIntake = 40;

    // Positions
    public static final double kIntakePos = 0;
    public static final double kParallelPos = 4;
    public static final double kStowedPos = 13.25;

    // Speeds
    public static final double kIntakePow = 0.75;
    public static final double kOuttakePow = -1;
    public static final double kStopPow = 0;
  }

  public final class IndexerConstants {
    public static final int kLeftIndexerMotorID = 6;
    public static final int kRightIndexerMotorID = 5;
    public static final int kBreakBeamID = 2;

    public static final int kSmartCurrentLimit = 40;

    // Speeds
    public static final double kIntakePow = 0.5;
    public static final double kOuttakePow = -0.5;
    public static final double kStopPow = 0;
  }

  public final class ClimberConstants {
    public static final int kClimberMotorID = 2;
    public static final int kCageMotorID = 7;

    // PID parameters
    public static final double kP = 0.028;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int kSmartCurrentLimit = 100;

    // Positions
    public static final double kHomePos = 0;
    public static final double kDeployPos = 172;
    public static final double kClimbPos = 411;
  }

  public final class AutoAlignConstants {
    public static final double X_REEF_ALIGNMENT_P = 1;
    public static final double Y_REEF_ALIGNMENT_P = 0.18;
    public static final double ROT_REEF_ALIGNMENT_P = 0.1;
  
    public static final double RIGHT_X_SETPOINT_REEF_ALIGNMENT = 0.11;  // TX - Vertical pose
    public static final double RIGHT_X_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double LEFT_X_SETPOINT_REEF_ALIGNMENT = -0.25;  // TX - Vertical pose
    public static final double LEFT_X_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double RIGHT_Y_SETPOINT_REEF_ALIGNMENT = -0.56;  // TZ - Horizontal pose
    public static final double RIGHT_Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double LEFT_Y_SETPOINT_REEF_ALIGNMENT = 0.53;  // TZ - Horizontal pose
    public static final double LEFT_Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double RIGHT_ROT_SETPOINT_REEF_ALIGNMENT = 3.24;  // RY - Rotation
    public static final double LEFT_ROT_SETPOINT_REEF_ALIGNMENT = 1.9;  // RY - Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
  
    public static final double DONT_SEE_TAG_WAIT_TIME = 3;
    public static final double POSE_VALIDATION_TIME = 0.3;
  }
}