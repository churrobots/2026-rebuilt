// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public class HardwareConstants {
  // CAN bus wiring order, traced on the robot (2026-10-13). CAN is one daisy chain:
  // the yellow/green wires go from the roboRIO to each device in turn, and the far end
  // must be terminated (PDH termination switch on, or a 120 ohm resistor).
  //
  //   1. roboRIO (built-in termination)
  //   2. SPARK MAX 14 spindexer: on CAN (connected in dumps). Light blinks magenta while
  //      disabled; that's coast mode + robot disabled, not a fault.
  //   3. Pigeon2: on CAN (light blinks orange on both sides at once = sees CAN, robot
  //      disabled). Tuner X shows it as ID 12; DriveConstants.pigeonCanId was 9 on this
  //      branch, so the code couldn't find it. Changed the code to 12 (2026-10-13).
  //   4. SPARK MAX 2 FrontRightTurn: on CAN (connected in dumps; ID 2 confirmed over USB in
  //      REV Hardware Client). Light blinks cyan = brake mode (drive code sets brake) +
  //      robot disabled.
  //   5. TalonFX 20 intake roller: on CAN and checked out good.
  //   6. SPARK Flex 6 FrontRightDrive: was NOT on CAN but powered, with a likely CAN fault
  //      light. After working on the CAN link between 5 and 6, no drive SPARK warns at
  //      startup (2026-10-12 19:49 run), so it appears to be back on CAN.
  //   7. SPARK MAX 11 intake arm: on CAN (answers since the CAN fix). Its setup crashed robot
  //      code with "Invalid parameter id" from SparkMax.configure because its firmware was
  //      older than our 2026 REVLib. Fixed by updating it to 2026 firmware (2026-10-13).
  //   8. SPARK Flex 7 FrontLeftDrive
  //   9. SPARK MAX 3 FrontLeftTurn
  //  10. TalonFX 15 shooter
  //  11. SPARK MAX 4 BackLeftTurn
  //  12. SPARK MAX 1 BackRightTurn
  //  13. SPARK Flex 5 BackRightDrive
  //  14. TalonFX 9 feeder
  //  15. SPARK Flex 8 BackLeftDrive
  //  16. Terminator (end of the chain)
  //
  // All 14 devices the code uses are on the chain. Robot code starts all the way up again
  // after the CAN fix and the intake arm firmware update (2026-10-13). The climber TalonFX
  // (CAN 10) isn't on the chain; the code doesn't use it right now.
  //
  // Seen in Phoenix Tuner X (2026-10-13): Pigeon2 ID 12, TalonFX ID 20 (intake roller).
  // Seen in REV Hardware Client over USB on SPARK 2 (2026-10-13): only one other SPARK on
  // CAN, ID 14 ("Unknown SPARK", it runs 2026 firmware; SPARK 2 still runs v25.0.4).
  // So before the CAN fix, SPARKs 1, 3-8 and 11 weren't on the bus at all, wrong IDs or not.
  //
  // Every CAN device the code expects, for reference while tracing:
  //   SPARK MAX  1 BackRightTurn     SPARK Flex  5 BackRightDrive    Pigeon2  12 gyro
  //   SPARK MAX  2 FrontRightTurn    SPARK Flex  6 FrontRightDrive   TalonFX   9 feeder
  //   SPARK MAX  3 FrontLeftTurn     SPARK Flex  7 FrontLeftDrive    TalonFX  10 climber (unused)
  //   SPARK MAX  4 BackLeftTurn      SPARK Flex  8 BackLeftDrive     TalonFX  15 shooter
  //   SPARK MAX 11 intake arm        SPARK MAX  14 spindexer         TalonFX  20 intake roller

  public static final int FEEDER_MOTOR_ID = 9;
  public static final int SPINDEXER_MOTOR_ID = 14;
  public static final int SHOOTER_MOTOR_ID = 15;
  public static final int INTAKE_ARM_MOTOR_ID = 11;
  public static final int INTAKE_ROLLERS_MOTOR_ID = 20;
  public static final int CLIMBER_MOTOR_ID = 10;

  // Set this to "true" if we're at a competition and seeing loop overruns or
  // inability to load the dashboard or AdvantageKit
  public static final boolean REDUCE_ROBORIO_RESOURCE_USAGE = true;
  public static final boolean ENABLE_DIAGNOSTIC_POSES = true;
}
