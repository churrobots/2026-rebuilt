package frc.robot.util;

import java.util.HashMap;
import java.util.List;

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

  private static final int MAX_VISION_RESULTS_TO_SAMPLE = 3;
  private static final long MAX_ALLOWABLE_VISION_LATENCY_MILLISECONDS = 60;
  private static final long MAX_ALLOWABLE_VISION_UNSEEN_SECONDS = 5;

  public static void registerHardware(String hardwareName, Object device) {
    if (device instanceof TalonFX) {
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
    if (includeMemoryMonitoring) {
      long allocatedMemoryInBytes = (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory());
      long presumableFreeMemoryInBytes = Runtime.getRuntime().maxMemory() - allocatedMemoryInBytes;
      SmartDashboard.putNumber("HardwareMonitor/freeMemory", presumableFreeMemoryInBytes);
    }
    for (var talon : talonMap.entrySet()) {
      SmartDashboard.putBoolean("HardwareMonitor/" + talon.getKey() + "/hasFaults",
          // TODO: refactor FaultMonitor method into this class for registerHardware idiom
          FaultMonitor.hasAnyDisconnectsOrFaults(talon.getValue()));
    }
    for (var spark : sparkMap.entrySet()) {
      SmartDashboard.putBoolean("HardwareMonitor/" + spark.getKey() + "/hasFaults",
          // TODO: refactor FaultMonitor method into this class for registerHardware idiom
          FaultMonitor.hasAnyDisconnectsOrFaults(spark.getValue()));
    }
    for (var pigeon : pigeonMap.entrySet()) {
      SmartDashboard.putBoolean("HardwareMonitor/" + pigeon.getKey() + "/hasFaults",
          // TODO: refactor FaultMonitor method into this class for registerHardware idiom
          FaultMonitor.hasAnyDisconnectsOrFaults(pigeon.getValue()));
    }
    for (var camera : cameraMap.entrySet()) {
      SmartDashboard.putBoolean("HardwareMonitor/" + camera.getKey() + "/hasFaults",
          _cameraHasFaults(camera.getValue()));
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

}
