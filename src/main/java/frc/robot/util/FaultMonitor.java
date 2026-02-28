package frc.robot.util;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;

/**
 * Useful for monitoring high-level status of motors and other hardware devices
 * on the robot. Likely use the periodic() method of your subsystems to
 * set a SmartDashboard value that can be visualized on the dashboard so
 * the drive team knows when a device is broken or disconnected during a match.
 */
public class FaultMonitor {

  public static boolean hasAnyDisconnectsOrFaults(Pigeon2 device) {
    if (!device.isConnected()) {
      return false;
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

  public static boolean hasAnyDisconnectsOrFaults(SparkBase device) {
    // TODO: check for supply voltage dropping out? how would we detect the white
    // wires being disconnected?
    if (device.hasActiveFault() || device.hasActiveWarning()) {
      return true;
    } else {
      return false;
    }
  }

  public static boolean hasAnyDisconnectsOrFaults(TalonFX device) {
    if (!device.isConnected()) {
      return false;
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
