package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
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

  // Firmware is only read once per device, the first time it shows up connected.
  private static final Map<String, String> firmwareCache = new HashMap<>();

  // Problems are logged when they appear, change or clear, so the robot console (and
  // FRC_UserProgram.log, which the debug data task downloads) keeps a history. A device that flaps is
  // logged at most once per interval; its latest state is logged when the interval ends.
  private static final double MIN_SECONDS_BETWEEN_LOGS = 1.0;
  private static final double MIN_SECONDS_BETWEEN_CAN_LOGS = 5.0;
  // CAN controllers count errors up to 255: 96+ is "error warning", 128+ is "error passive".
  private static final int CAN_ERROR_WARNING_LEVEL = 96;
  private static final int CAN_ERROR_PASSIVE_LEVEL = 128;
  private static final Map<String, String> loggedProblems = new HashMap<>();
  private static final Map<String, Double> lastLogSeconds = new HashMap<>();
  private static String loggedCanErrorLevel = "ok";
  private static int loggedBusOffCount = 0;
  private static int loggedTxFullCount = 0;
  private static double lastCanLogSeconds = Double.NEGATIVE_INFINITY;

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
    // Only run every 10th tick to reduce load
    tickCounter++;
    tickCounter = tickCounter % 10;
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
    for (var spark : sparkMap.entrySet()) {
      SmartDashboard.putBoolean("HardwareMonitor/FaultStatus/" + spark.getKey(),
          // TODO: refactor FaultMonitor method into this class for registerHardware idiom
          !_hasAnyDisconnectsOrFaults(spark.getValue()));
    }
    for (var camera : cameraMap.entrySet()) {
      SmartDashboard.putBoolean("HardwareMonitor/FaultStatus/" + camera.getKey(),
          !_cameraHasFaults(camera.getValue()));
    }
    _publishDeviceStatus();
  }

  /** One device's status, published under HardwareMonitor/Devices/<name>/. */
  private record DeviceStatus(String name, String type, int canId, boolean connected,
      List<String> faults, List<String> warnings, String firmware) {
  }

  /**
   * Publishes every device's type, CAN ID, connection, named faults and firmware under
   * "HardwareMonitor/Devices/<name>/", plus CAN bus health numbers ("HardwareMonitor/CANBus/*").
   * Also sets the TalonFX and Pigeon2 FaultStatus lights, so their faults are only read once.
   */
  private static void _publishDeviceStatus() {
    Map<String, DeviceStatus> devices = new TreeMap<>();
    for (String name : nullHardware) {
      devices.put(name, new DeviceStatus(name, "unknown", -1, false,
          List.of("NotCreated"), List.of(), ""));
    }
    for (var talon : talonMap.entrySet()) {
      devices.put(talon.getKey(), _ctreStatus(talon.getKey(), "TalonFX", talon.getValue(),
          _talonFaults(talon.getValue()), talon.getValue().getVersion()));
    }
    for (var pigeon : pigeonMap.entrySet()) {
      devices.put(pigeon.getKey(), _ctreStatus(pigeon.getKey(), "Pigeon2", pigeon.getValue(),
          _pigeonFaults(pigeon.getValue()), pigeon.getValue().getVersion()));
    }
    for (String name : talonMap.keySet()) {
      _publishCtreFaultStatus(devices.get(name));
    }
    for (String name : pigeonMap.keySet()) {
      _publishCtreFaultStatus(devices.get(name));
    }
    for (var spark : sparkMap.entrySet()) {
      devices.put(spark.getKey(), _sparkStatus(spark.getKey(), spark.getValue()));
    }
    for (var camera : cameraMap.entrySet()) {
      devices.put(camera.getKey(), new DeviceStatus(camera.getKey(), "PhotonCamera", -1,
          camera.getValue().isConnected(), List.of(), List.of(), ""));
    }

    for (DeviceStatus device : devices.values()) {
      String prefix = "HardwareMonitor/Devices/" + device.name() + "/";
      SmartDashboard.putString(prefix + "type", device.type());
      SmartDashboard.putNumber(prefix + "canId", device.canId());
      SmartDashboard.putBoolean(prefix + "connected", device.connected());
      SmartDashboard.putStringArray(prefix + "faults", device.faults().toArray(String[]::new));
      SmartDashboard.putStringArray(prefix + "warnings", device.warnings().toArray(String[]::new));
      SmartDashboard.putString(prefix + "firmware", device.firmware());
      _logProblemChanges(device);
    }

    // Error counts climb when the CAN chain is broken or unterminated.
    CANStatus can = RobotController.getCANStatus();
    SmartDashboard.putNumber("HardwareMonitor/CANBus/utilizationPercent", can.percentBusUtilization * 100);
    SmartDashboard.putNumber("HardwareMonitor/CANBus/busOffCount", can.busOffCount);
    SmartDashboard.putNumber("HardwareMonitor/CANBus/txFullCount", can.txFullCount);
    SmartDashboard.putNumber("HardwareMonitor/CANBus/receiveErrorCount", can.receiveErrorCount);
    SmartDashboard.putNumber("HardwareMonitor/CANBus/transmitErrorCount", can.transmitErrorCount);
    _logCanBusChanges(can);
  }

  /** Logs when a device's problem appears, changes or clears. */
  private static void _logProblemChanges(DeviceStatus device) {
    String problem;
    if (!device.connected()) {
      problem = "DISCONNECTED";
    } else if (!device.faults().isEmpty() || !device.warnings().isEmpty()) {
      List<String> issues = new ArrayList<>(device.faults());
      for (String warning : device.warnings()) {
        issues.add("warning " + warning);
      }
      problem = "faults: " + String.join(", ", issues);
    } else {
      problem = "";
    }

    String logged = loggedProblems.getOrDefault(device.name(), "");
    double now = Timer.getFPGATimestamp();
    if (problem.equals(logged)
        || now - lastLogSeconds.getOrDefault(device.name(), Double.NEGATIVE_INFINITY) < MIN_SECONDS_BETWEEN_LOGS) {
      return;
    }
    loggedProblems.put(device.name(), problem);
    lastLogSeconds.put(device.name(), now);
    String label = String.format("[HardwareMonitor %.1fs] %s (%s%s)", now, device.name(), device.type(),
        device.canId() < 0 ? "" : " CAN " + device.canId());
    if (problem.isEmpty()) {
      System.out.println(label + ": OK again (was " + logged + ")");
    } else {
      DriverStation.reportWarning(label + ": " + problem, false);
    }
  }

  /** Logs bus-off and full-transmit-buffer events, and when CAN error counts cross a level. */
  private static void _logCanBusChanges(CANStatus can) {
    int worstErrorCount = Math.max(can.receiveErrorCount, can.transmitErrorCount);
    String errorLevel = worstErrorCount >= CAN_ERROR_PASSIVE_LEVEL ? "error passive"
        : worstErrorCount >= CAN_ERROR_WARNING_LEVEL ? "error warning" : "ok";
    boolean busOff = can.busOffCount > loggedBusOffCount;
    boolean txFull = can.txFullCount > loggedTxFullCount;
    double now = Timer.getFPGATimestamp();
    if ((!busOff && !txFull && errorLevel.equals(loggedCanErrorLevel))
        || now - lastCanLogSeconds < MIN_SECONDS_BETWEEN_CAN_LOGS) {
      return;
    }
    String message = String.format(
        "[HardwareMonitor %.1fs] CAN bus %s: rx errors %d, tx errors %d, bus-off +%d, tx-full +%d, %.0f%% used",
        now, errorLevel, can.receiveErrorCount, can.transmitErrorCount,
        can.busOffCount - loggedBusOffCount, can.txFullCount - loggedTxFullCount,
        can.percentBusUtilization * 100);
    if (errorLevel.equals("ok") && !busOff && !txFull) {
      System.out.println(message);
    } else {
      DriverStation.reportWarning(message, false);
    }
    loggedCanErrorLevel = errorLevel;
    loggedBusOffCount = can.busOffCount;
    loggedTxFullCount = can.txFullCount;
    lastCanLogSeconds = now;
  }

  /** CTRE devices are healthy when connected with no active faults. */
  private static void _publishCtreFaultStatus(DeviceStatus device) {
    SmartDashboard.putBoolean("HardwareMonitor/FaultStatus/" + device.name(),
        device.connected() && device.faults().isEmpty());
  }

  private static DeviceStatus _ctreStatus(String name, String type, ParentDevice device,
      List<String> faults, StatusSignal<Integer> versionSignal) {
    boolean connected = device.isConnected();
    if (connected && !firmwareCache.containsKey(name)) {
      int version = versionSignal.getValue();
      firmwareCache.put(name, ((version >> 24) & 0xff) + "." + ((version >> 16) & 0xff) + "."
          + ((version >> 8) & 0xff) + "." + (version & 0xff));
    }
    return new DeviceStatus(name, type, device.getDeviceID(), connected,
        connected ? faults : List.of(), List.of(), firmwareCache.getOrDefault(name, ""));
  }

  private static DeviceStatus _sparkStatus(String name, SparkBase device) {
    // REVLib has no isConnected(); a read that times out is how SparkUtil spots a missing SPARK.
    device.getBusVoltage();
    boolean connected = device.getLastError() == REVLibError.kOk;
    List<String> faults = new ArrayList<>();
    List<String> warnings = new ArrayList<>();
    if (connected) {
      SparkBase.Faults f = device.getFaults();
      _addIf(faults, f.other, "Other");
      _addIf(faults, f.motorType, "MotorType");
      _addIf(faults, f.sensor, "Sensor");
      _addIf(faults, f.can, "CAN");
      _addIf(faults, f.temperature, "Temperature");
      _addIf(faults, f.gateDriver, "GateDriver");
      _addIf(faults, f.escEeprom, "EscEeprom");
      _addIf(faults, f.firmware, "Firmware");
      SparkBase.Warnings w = device.getWarnings();
      _addIf(warnings, w.brownout, "Brownout");
      _addIf(warnings, w.overcurrent, "Overcurrent");
      _addIf(warnings, w.escEeprom, "EscEeprom");
      _addIf(warnings, w.extEeprom, "ExtEeprom");
      _addIf(warnings, w.sensor, "Sensor");
      _addIf(warnings, w.stall, "Stall");
      _addIf(warnings, w.hasReset, "HasReset");
      _addIf(warnings, w.other, "Other");
      if (!firmwareCache.containsKey(name)) {
        firmwareCache.put(name, device.getFirmwareString());
      }
    }
    String type = device.getClass().getSimpleName();
    return new DeviceStatus(name, type, device.getDeviceId(), connected, faults, warnings,
        firmwareCache.getOrDefault(name, ""));
  }

  private static void _addIf(List<String> names, boolean active, String name) {
    if (active) {
      names.add(name);
    }
  }

  /** Names of the active faults in a list of (name, signal) pairs. */
  private static List<String> _activeFaults(Object... namesAndSignals) {
    List<String> active = new ArrayList<>();
    for (int i = 0; i < namesAndSignals.length; i += 2) {
      if (Boolean.TRUE.equals(((StatusSignal<?>) namesAndSignals[i + 1]).getValue())) {
        active.add((String) namesAndSignals[i]);
      }
    }
    return active;
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

  private static List<String> _pigeonFaults(Pigeon2 device) {
    return _activeFaults(
        "Hardware", device.getFault_Hardware(),
        "Undervoltage", device.getFault_Undervoltage(),
        "BootDuringEnable", device.getFault_BootDuringEnable(),
        "UnlicensedFeatureInUse", device.getFault_UnlicensedFeatureInUse(),
        "BootupAccelerometer", device.getFault_BootupAccelerometer(),
        "BootupGyroscope", device.getFault_BootupGyroscope(),
        "BootupMagnetometer", device.getFault_BootupMagnetometer(),
        "BootIntoMotion", device.getFault_BootIntoMotion(),
        "DataAcquiredLate", device.getFault_DataAcquiredLate(),
        "LoopTimeSlow", device.getFault_LoopTimeSlow(),
        "SaturatedMagnetometer", device.getFault_SaturatedMagnetometer(),
        "SaturatedAccelerometer", device.getFault_SaturatedAccelerometer(),
        "SaturatedGyroscope", device.getFault_SaturatedGyroscope());
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

  private static List<String> _talonFaults(TalonFX device) {
    return _activeFaults(
        "Hardware", device.getFault_Hardware(),
        "ProcTemp", device.getFault_ProcTemp(),
        "DeviceTemp", device.getFault_DeviceTemp(),
        "Undervoltage", device.getFault_Undervoltage(),
        "BootDuringEnable", device.getFault_BootDuringEnable(),
        "UnlicensedFeatureInUse", device.getFault_UnlicensedFeatureInUse(),
        "BridgeBrownout", device.getFault_BridgeBrownout(),
        "RemoteSensorReset", device.getFault_RemoteSensorReset(),
        "MissingDifferentialFX", device.getFault_MissingDifferentialFX(),
        "RemoteSensorPosOverflow", device.getFault_RemoteSensorPosOverflow(),
        "OverSupplyV", device.getFault_OverSupplyV(),
        "UnstableSupplyV", device.getFault_UnstableSupplyV(),
        "ReverseHardLimit", device.getFault_ReverseHardLimit(),
        "ForwardHardLimit", device.getFault_ForwardHardLimit(),
        "ReverseSoftLimit", device.getFault_ReverseSoftLimit(),
        "ForwardSoftLimit", device.getFault_ForwardSoftLimit(),
        "MissingSoftLimitRemote", device.getFault_MissingSoftLimitRemote(),
        "MissingHardLimitRemote", device.getFault_MissingHardLimitRemote(),
        "RemoteSensorDataInvalid", device.getFault_RemoteSensorDataInvalid(),
        "FusedSensorOutOfSync", device.getFault_FusedSensorOutOfSync(),
        "StatorCurrLimit", device.getFault_StatorCurrLimit(),
        "SupplyCurrLimit", device.getFault_SupplyCurrLimit(),
        "UsingFusedCANcoderWhileUnlicensed", device.getFault_UsingFusedCANcoderWhileUnlicensed(),
        "StaticBrakeDisabled", device.getFault_StaticBrakeDisabled());
  }

}
