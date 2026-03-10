package frc.robot.util;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Useful for monitoring high-level status of motors and other hardware devices
 * on the robot. Likely use the periodic() method of your subsystems to
 * set a SmartDashboard value that can be visualized on the dashboard so
 * the drive team knows when a device is broken or disconnected during a match.
 */
public class HardwareMonitor {

  private static final HashMap<String, PhotonCamera> cameraMap = new HashMap<>();
  private static final HashMap<String, TalonFX> talonMap = new HashMap<>();
  private static final HashMap<String, SparkBase> sparkMap = new HashMap<>();
  private static final HashMap<String, Pigeon2> pigeonMap = new HashMap<>();
  private static final Set<String> nullHardware = new HashSet<>();

  private static final int MAX_VISION_RESULTS_TO_SAMPLE = 3;
  private static final long MAX_ALLOWABLE_VISION_LATENCY_MILLISECONDS = 60;
  private static final long MAX_ALLOWABLE_VISION_UNSEEN_SECONDS = 5;
  private static int tickCounter = 0;

  public static void registerHardware(String hardwareName, Object device) {
    if (device == null) {
      nullHardware.add(hardwareName);
    } else if (device instanceof TalonFX) {
      _registerTalonFX(hardwareName, (TalonFX) device);
    } else if (device instanceof SparkBase) {
      _registerSparkBase(hardwareName, (SparkBase) device);
    } else if (device instanceof Pigeon2) {
      _registerPigeon2(hardwareName, (Pigeon2) device);
    } else if (device instanceof PhotonCamera) {
      _registerPhotonCamera(hardwareName, (PhotonCamera) device);
    } else {
      throw new IllegalArgumentException("Unknown device type: " + device.getClass().getName());
    }
  }

  public static void dumpHardwareStatusToNetworkTables(boolean includeMemoryMonitoring) {
    // Only run every 5th tick to reduce load
    tickCounter++;
    tickCounter = tickCounter % 5;
    if (tickCounter != 0) {
      return;
    }
    if (includeMemoryMonitoring) {
      long allocatedMemoryInBytes = (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory());
      long presumableFreeMemoryInBytes = Runtime.getRuntime().maxMemory() - allocatedMemoryInBytes;
      SmartDashboard.putNumber("HardwareMonitor/freeMemory", presumableFreeMemoryInBytes);
    }
    for (String nullHardwareName : nullHardware) {
      // Nulled hardware objects are automatically faulted
      SmartDashboard.putBoolean("HardwareMonitor/FaultStatus/" + nullHardwareName, false);
    }
    for (var talon : talonMap.entrySet()) {
      SmartDashboard.putBoolean("HardwareMonitor/FaultStatus/" + talon.getKey(),
          // TODO: refactor FaultMonitor method into this class for registerHardware idiom
          !_hasAnyDisconnectsOrFaults(talon.getValue()));
    }
    for (var spark : sparkMap.entrySet()) {
      SmartDashboard.putBoolean("HardwareMonitor/FaultStatus/" + spark.getKey(),
          // TODO: refactor FaultMonitor method into this class for registerHardware idiom
          !_hasAnyDisconnectsOrFaults(spark.getValue()));
    }
    for (var pigeon : pigeonMap.entrySet()) {
      SmartDashboard.putBoolean("HardwareMonitor/FaultStatus/" + pigeon.getKey(),
          // TODO: refactor FaultMonitor method into this class for registerHardware idiom
          !_hasAnyDisconnectsOrFaults(pigeon.getValue()));
    }
    for (var camera : cameraMap.entrySet()) {
      SmartDashboard.putBoolean("HardwareMonitor/FaultStatus/" + camera.getKey(),
          !_cameraHasFaults(camera.getValue()));
    }
  }

  private static void _registerPhotonCamera(String hardwareName, PhotonCamera camera) {
    cameraMap.put(hardwareName, camera);
  }

  private static void _registerTalonFX(String hardwareName, TalonFX talon) {
    talonMap.put(hardwareName, talon);
  }

  private static void _registerSparkBase(String hardwareName, SparkBase spark) {
    sparkMap.put(hardwareName, spark);
  }

  private static void _registerPigeon2(String hardwareName, Pigeon2 pigeon) {
    pigeonMap.put(hardwareName, pigeon);
  }

  private static boolean _cameraHasFaults(PhotonCamera camera) {
    if (camera == null) {
      return true;
    }
    boolean hasConnection = camera.isConnected();
    long maxAllowableTimeSinceLastPong = MAX_ALLOWABLE_VISION_UNSEEN_SECONDS * 1000000L;
    List<PhotonPipelineResult> allResults = camera.getAllUnreadResults();
    List<PhotonPipelineResult> resultsSample = allResults.subList(0,
        Math.min(MAX_VISION_RESULTS_TO_SAMPLE, allResults.size()));
    boolean hasHighLatency = false;
    boolean longTimeSinceLastPong = false;
    for (PhotonPipelineResult result : resultsSample) {
      if (result.metadata.timeSinceLastPong > maxAllowableTimeSinceLastPong) {
        longTimeSinceLastPong = true;
      }
      if (result.metadata.getLatencyMillis() > MAX_ALLOWABLE_VISION_LATENCY_MILLISECONDS) {
        hasHighLatency = true;
        break;
      }
    }
    return !hasConnection || hasHighLatency || longTimeSinceLastPong;
  }

  private static boolean _hasAnyDisconnectsOrFaults(Pigeon2 device) {
    if (!device.isConnected()) {
      return true;
    } else {
      boolean faultHardware = device.getFault_Hardware().getValue() == true;
      boolean faultUndervoltage = device.getFault_Undervoltage().getValue() == true;
      boolean faultBootDuringEnable = device.getFault_BootDuringEnable().getValue() == true;
      boolean faultUnlicensedFeatureInUse = device.getFault_UnlicensedFeatureInUse().getValue() == true;
      boolean faultBootupAccelerometer = device.getFault_BootupAccelerometer().getValue() == true;
      boolean faultBootupGyroscope = device.getFault_BootupGyroscope().getValue() == true;
      boolean faultBootupMagnetometer = device.getFault_BootupMagnetometer().getValue() == true;
      boolean faultBootIntoMotion = device.getFault_BootIntoMotion().getValue() == true;
      boolean faultDataAcquiredLate = device.getFault_DataAcquiredLate().getValue() == true;
      boolean faultLoopTimeSlow = device.getFault_LoopTimeSlow().getValue() == true;
      boolean faultSaturatedMagnetometer = device.getFault_SaturatedMagnetometer().getValue() == true;
      boolean faultSaturatedAccelerometer = device.getFault_SaturatedAccelerometer().getValue() == true;
      boolean faultSaturatedGyroscope = device.getFault_SaturatedGyroscope().getValue() == true;
      boolean hasAnyFaults = faultHardware || faultUndervoltage || faultBootDuringEnable
          || faultUnlicensedFeatureInUse || faultBootupAccelerometer || faultBootupGyroscope
          || faultBootupMagnetometer || faultBootIntoMotion || faultDataAcquiredLate || faultLoopTimeSlow
          || faultSaturatedMagnetometer || faultSaturatedAccelerometer || faultSaturatedGyroscope;
      if (hasAnyFaults) {
        return true;
      } else {
        return false;
      }
    }
  }

  private static boolean _hasAnyDisconnectsOrFaults(SparkBase device) {
    // TODO: check for supply voltage dropping out? how would we detect the white
    // wires being disconnected?
    if (device.hasActiveFault() || device.hasActiveWarning()) {
      return true;
    } else {
      return false;
    }
  }

  private static boolean _hasAnyDisconnectsOrFaults(TalonFX device) {
    if (!device.isConnected()) {
      return true;
    } else {
      boolean faultHardware = device.getFault_Hardware().getValue() == true;
      boolean faultProcTemp = device.getFault_ProcTemp().getValue() == true;
      boolean faultDeviceTemp = device.getFault_DeviceTemp().getValue() == true;
      boolean faultUndervoltage = device.getFault_Undervoltage().getValue() == true;
      boolean faultBootDuringEnable = device.getFault_BootDuringEnable().getValue() == true;
      boolean faultUnlicensedFeatureInUse = device.getFault_UnlicensedFeatureInUse().getValue() == true;
      boolean faultBridgeBrownout = device.getFault_BridgeBrownout().getValue() == true;
      boolean faultRemoteSensorReset = device.getFault_RemoteSensorReset().getValue() == true;
      boolean faultMissingDifferentialFX = device.getFault_MissingDifferentialFX().getValue() == true;
      boolean faultRemoteSensorPosOverflow = device.getFault_RemoteSensorPosOverflow().getValue() == true;
      boolean faultOverSupplyV = device.getFault_OverSupplyV().getValue() == true;
      boolean faultUnstableSupplyV = device.getFault_UnstableSupplyV().getValue() == true;
      boolean faultReverseHardLimit = device.getFault_ReverseHardLimit().getValue() == true;
      boolean faultForwardHardLimit = device.getFault_ForwardHardLimit().getValue() == true;
      boolean faultReverseSoftLimit = device.getFault_ReverseSoftLimit().getValue() == true;
      boolean faultForwardSoftLimit = device.getFault_ForwardSoftLimit().getValue() == true;
      boolean faultMissingSoftLimitRemote = device.getFault_MissingSoftLimitRemote().getValue() == true;
      boolean faultMissingHardLimitRemote = device.getFault_MissingHardLimitRemote().getValue() == true;
      boolean faultRemoteSensorDataInvalid = device.getFault_RemoteSensorDataInvalid().getValue() == true;
      boolean faultFusedSensorOutOfSync = device.getFault_FusedSensorOutOfSync().getValue() == true;
      boolean faultStatorCurrLimit = device.getFault_StatorCurrLimit().getValue() == true;
      boolean faultSupplyCurrLimit = device.getFault_SupplyCurrLimit().getValue() == true;
      boolean faultUsingFusedCANcoderWhileUnlicensed = device.getFault_UsingFusedCANcoderWhileUnlicensed()
          .getValue() == true;
      boolean faultStaticBrakeDisabled = device.getFault_StaticBrakeDisabled().getValue() == true;
      boolean hasAnyFaults = faultHardware || faultProcTemp || faultDeviceTemp || faultUndervoltage
          || faultBootDuringEnable || faultUnlicensedFeatureInUse || faultBridgeBrownout || faultRemoteSensorReset
          || faultMissingDifferentialFX || faultRemoteSensorPosOverflow || faultOverSupplyV || faultUnstableSupplyV
          || faultReverseHardLimit || faultForwardHardLimit || faultReverseSoftLimit || faultForwardSoftLimit
          || faultMissingSoftLimitRemote || faultMissingHardLimitRemote || faultRemoteSensorDataInvalid
          || faultFusedSensorOutOfSync || faultStatorCurrLimit || faultSupplyCurrLimit
          || faultUsingFusedCANcoderWhileUnlicensed || faultStaticBrakeDisabled;
      if (hasAnyFaults) {
        return true;
      } else {
        return false;
      }
    }
  }

}
