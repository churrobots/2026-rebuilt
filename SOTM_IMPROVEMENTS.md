# Shoot-On-The-Move System: Improvements & Testing Plan

## 🎯 System Overview

Your SOTM system uses Newton's method to solve for the correct aim point and RPM while the robot is moving. It accounts for:
- Robot velocity (inherited by the projectile)
- Drag on the ball during flight
- Processing latency
- Launcher position offset
- Acceleration prediction

## 📋 Phase 1: Complete Integration (Before Any Testing)

### 1.1 Integrate SOTM RPM with Shooter

**Current State:** SOTM calculates RPM but shooter doesn't use it.

**File:** `RobotContainer.java`

**Add new method:**
```java
/**
 * Auto-shoot command with SOTM-calculated RPM and turret aim.
 * Uses shoot-on-the-move for both velocity and angle compensation.
 */
public Command autoShootWithSOTM() {
  return Commands.parallel(
      // Use SOTM RPM instead of distance-based lookup
      shooter.setVelocity(() -> {
        if (Constants.CHURRET_ENABLED && semiAutoHelper != null) {
          ShotCalculator.LaunchParameters shot = semiAutoHelper.calculateSOTMShot();
          return RPM.of(shot.rpm());
        }
        return SemiAutoHelper.getShooterVelocityForHubDistance(drive);
      }),
      // Aim turret with SOTM
      churret.aimWithSOTM(semiAutoHelper, drive),
      // Aim drive with SOTM angle
      DriveCommands.joystickDriveAtAngle(
          drive,
          () -> -controller.getLeftY(),
          () -> -controller.getLeftX(),
          () -> {
            ShotCalculator.LaunchParameters shot = semiAutoHelper.calculateSOTMShot();
            return shot.driveAngle();
          },
          () -> false)
  );
}
```

**Bind to button:**
```java
// In bindCommandsForTeleop(), add:
if (Constants.CHURRET_ENABLED && churret != null) {
  // Start button: Full SOTM auto-shoot
  controller.start().whileTrue(autoShootWithSOTM());
}
```

### 1.2 Add Comprehensive Telemetry

**File:** `SemiAutoHelper.java`

**Add method:**
```java
/**
 * Publishes detailed SOTM telemetry to SmartDashboard for debugging.
 * Call this in periodic() or after calculateSOTMShot().
 */
public void publishSOTMTelemetry() {
  ShotCalculator.LaunchParameters shot = calculateSOTMShot();
  ChassisSpeeds fieldVel = drive.getFieldVelocity();

  // Core SOTM outputs
  SmartDashboard.putNumber("SOTM/RPM", shot.rpm());
  SmartDashboard.putNumber("SOTM/DriveAngleDeg", shot.driveAngle().getDegrees());
  SmartDashboard.putNumber("SOTM/TOF", shot.timeOfFlightSec());
  SmartDashboard.putNumber("SOTM/Confidence", shot.confidence());
  SmartDashboard.putBoolean("SOTM/Valid", shot.isValid());
  SmartDashboard.putNumber("SOTM/SolvedDistance", shot.solvedDistanceM());
  SmartDashboard.putNumber("SOTM/Iterations", shot.iterationsUsed());
  SmartDashboard.putBoolean("SOTM/WarmStart", shot.warmStartUsed());

  // Robot state
  SmartDashboard.putNumber("SOTM/RobotSpeedMPS",
      Math.hypot(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond));
  SmartDashboard.putNumber("SOTM/RobotVelX", fieldVel.vxMetersPerSecond);
  SmartDashboard.putNumber("SOTM/RobotVelY", fieldVel.vyMetersPerSecond);
  SmartDashboard.putNumber("SOTM/RobotOmega", fieldVel.omegaRadiansPerSecond);

  // Distance to target
  SmartDashboard.putNumber("SOTM/DistanceToHub",
      getDistanceToHub(drive).in(Meters));
}
```

**Call in RobotContainer.periodic():**
```java
@Override
public void robotPeriodic() {
  if (Constants.CHURRET_ENABLED && semiAutoHelper != null) {
    semiAutoHelper.publishSOTMTelemetry();
  }
}
```

### 1.3 Add LUT Generation Status

**File:** `SemiAutoHelper.java`

**Update constructor:**
```java
public SemiAutoHelper(final Drive drive) {
  this.drive = drive;

  System.out.println("=== SOTM LUT Generation Started ===");
  long startTime = System.currentTimeMillis();

  GeneratedLUT shootOnTheMoveLookupTable = createShootOnTheMoveLookupTable();

  long elapsed = System.currentTimeMillis() - startTime;
  System.out.println("✓ LUT Generated in " + elapsed + "ms");
  System.out.println("  - Reachable entries: " + shootOnTheMoveLookupTable.reachableCount());
  System.out.println("  - Max range: " + String.format("%.2f", shootOnTheMoveLookupTable.maxRangeM()) + "m");
  System.out.println("=== SOTM Ready ===");

  this.shotCalculator = setupShotCalculator(shootOnTheMoveLookupTable);
}
```

---

## 📏 Phase 2: Measure Your Robot

### 2.1 CAD Measurements (Do This First!)

Open your robot CAD and measure these **precisely**:

| Parameter | Location | How to Measure | Units |
|-----------|----------|----------------|-------|
| `exitHeightM` | Shooter exit point | Height from floor to ball release | meters |
| `launcherOffsetX` | Shooter location | Distance forward from robot center | meters |
| `launcherOffsetY` | Shooter location | Distance left from robot center | meters |
| `launchAngleDeg` | Shooter angle | Angle from horizontal | degrees |
| `wheelDiameterM` | Shooter wheels | Actual worn wheel diameter | meters |

**How to find robot center:**
- It's the center of the swerve module square
- Typically the center of rotation when spinning in place

### 2.2 Physical Measurements (After Build)

With the actual robot:

1. **Wheel Diameter:**
   ```
   - Measure with calipers (worn wheels shrink!)
   - Measure multiple points, average them
   - Convert to meters (divide inches by 39.37)
   ```

2. **Exit Height:**
   ```
   - Measure from floor to center of ball at release
   - Account for carpet compression
   ```

3. **Launch Angle:**
   ```
   - Use a protractor or angle finder
   - Measure the actual trajectory angle, not hood angle
   ```

### 2.3 Update Configuration

**File:** `SemiAutoHelper.java` - Line 239-269

Update with your actual measurements:
```java
SimParameters params = new SimParameters(
    0.215,    // ball mass (from game manual)
    0.1501,   // ball diameter (from game manual)
    0.47,     // drag coefficient (tune if needed)
    0.2,      // Magnus coefficient (tune if needed)
    1.225,    // air density (adjust for venue altitude)
    YOUR_MEASURED_EXIT_HEIGHT,      // ← UPDATE THIS
    YOUR_MEASURED_WHEEL_DIAMETER,   // ← UPDATE THIS
    1.83,     // target height (from game manual)
    0.6,      // slip factor (tune in Phase 4)
    YOUR_MEASURED_LAUNCH_ANGLE,     // ← UPDATE THIS
    0.001,    // timestep (don't change)
    1500,     // min RPM (adjust to your shooter)
    6000,     // max RPM (adjust to your shooter)
    25,       // iterations (don't change)
    5.0       // max sim time (don't change)
);
```

**File:** `SemiAutoHelper.java` - Line 268-269

```java
config.launcherOffsetX = YOUR_MEASURED_OFFSET_X;  // ← UPDATE THIS
config.launcherOffsetY = YOUR_MEASURED_OFFSET_Y;  // ← UPDATE THIS
```

---

## 🧪 Phase 3: Simulation Testing

### 3.1 Setup Sim Environment

1. **Enable CHURRET:**
   ```java
   // Constants.java
   public static final boolean CHURRET_ENABLED = true;
   ```

2. **Start sim:**
   ```bash
   ./gradlew simulateJava
   ```

3. **Open visualization tools:**
   - Glass (for field view)
   - AdvantageScope (for plotting)
   - SmartDashboard (for telemetry)

### 3.2 Test Sequence (In Sim)

#### Test 1: Static Shots (Not Moving)
**Goal:** Verify LUT is reasonable

1. Enable teleop
2. Drive to various distances from hub (1m, 2m, 3m, 4m)
3. Stop completely
4. Press POV Right (SOTM aim)
5. Check SmartDashboard:
   ```
   SOTM/Confidence → should be 95-100
   SOTM/Valid → should be true
   SOTM/Iterations → should be 0-2 (static shot)
   SOTM/RPM → check if reasonable for distance
   ```

#### Test 2: Slow Moving Shots
**Goal:** Verify velocity compensation works

1. Drive slowly toward hub (~0.5 m/s)
2. Press POV Right (SOTM aim)
3. Observe:
   ```
   Churret/TargetAngle → should lead the target
   SOTM/Iterations → should be 2-5
   SOTM/Confidence → should be 70-90
   ```

4. Compare to static aim:
   - Press Left Bumper (static aim)
   - Note target angle
   - Press POV Right (SOTM)
   - SOTM angle should be different (compensating)

#### Test 3: Fast Moving Shots
**Goal:** Test speed limits

1. Drive fast (~2 m/s)
2. Press POV Right
3. Check confidence drops but stays valid
4. Drive very fast (>3 m/s)
5. SOTM/Valid should go false (speed cap)

#### Test 4: Convergence Testing
**Goal:** Verify Newton solver

1. Drive in various patterns (circles, figure-8)
2. Monitor `SOTM/Iterations`:
   - 2-4 iterations = good convergence
   - >10 iterations = solver struggling
   - If often >10, reduce `maxSOTMSpeed` in config

### 3.3 Sim Testing Checklist

```
□ LUT generates without errors (<2 seconds)
□ Static shots: confidence >95
□ Moving shots (0.5 m/s): confidence >70
□ Moving shots (2.0 m/s): confidence >50
□ Very fast (>3 m/s): correctly rejects (valid=false)
□ Solver converges in <5 iterations (typical)
□ Turret leads target when moving
□ No exceptions or crashes
□ Telemetry updates every cycle
```

---

## 🤖 Phase 4: Physical Robot Testing

### 4.1 Pre-Flight Checks

**Before first robot test:**

1. ✅ All CAD measurements entered
2. ✅ Sim tests passed
3. ✅ Turret hardware tested separately
4. ✅ Shooter RPM control tested separately
5. ✅ Vision system calibrated (if using)

### 4.2 Test Environment Setup

**Field setup:**
- Mark distances: 1m, 2m, 3m, 4m from hub
- Have spotter watching hub for safety
- Clear area for moving tests

**Safety:**
- Start with low speeds
- Have E-stop ready
- Test turret range of motion first
- Verify mechanical hard stops

### 4.3 Robot Test Sequence

#### Phase 4A: Static Shot Tuning

**Goal:** Tune slip factor

1. **Baseline Test:**
   ```
   - Place robot at exactly 2.0m from hub
   - Enable robot, teleoperate
   - Press POV Right (SOTM aim)
   - Record: SOTM/RPM value
   - Take 5 shots
   - Measure actual landing distance
   ```

2. **Tune Slip Factor:**
   ```
   If shots fall SHORT:
     - Increase slipFactor (try 0.65, 0.70)
     - Ball isn't getting full wheel speed

   If shots go LONG:
     - Decrease slipFactor (try 0.55, 0.50)
     - Ball getting more speed than expected

   If shots are PERFECT:
     - You're done! Record the slip factor value
   ```

3. **Repeat at multiple distances:**
   - 1.5m, 2.5m, 3.5m
   - Slip factor should be consistent
   - If it varies, average them

#### Phase 4B: Moving Shot Testing

**Goal:** Verify SOTM compensation

1. **Slow Approach Test:**
   ```
   - Start 4m from hub
   - Drive slowly toward hub (~0.3 m/s)
   - Press POV Right
   - Take shot at 2m mark
   - Record: did ball go in?
   ```

2. **Faster Approach:**
   ```
   - Increase speed to 0.8 m/s
   - Repeat test
   - Check confidence stays >60
   ```

3. **Strafing Test:**
   ```
   - Drive parallel to hub (strafing)
   - Speed: 0.5 m/s
   - Distance: 2m
   - Take shot
   - Turret should lead significantly
   ```

#### Phase 4C: Latency Tuning

**Goal:** Optimize latency compensation

1. **Baseline:**
   ```
   Current: phaseDelayMs = 30.0
            mechLatencyMs = 20.0
   ```

2. **If shots are BEHIND target (lag):**
   ```
   Increase phaseDelayMs by 10ms
   Test again
   ```

3. **If shots are AHEAD of target (over-compensating):**
   ```
   Decrease phaseDelayMs by 10ms
   Test again
   ```

### 4.4 Physical Testing Checklist

```
□ Static shots accurate at all distances
□ Slip factor tuned (within 0.05)
□ Moving shots (0.5 m/s) work
□ Moving shots (1.0 m/s) work
□ Strafing shots work
□ Confidence scores match actual performance
□ No mechanical issues (turret hits limits, etc.)
□ Latency compensation tuned
□ Driver can use SOTM effectively
□ System handles loss of vision gracefully
```

---

## 🎯 Phase 5: Match Optimization

### 5.1 In-Match Tuning

**During practice matches:**

1. **RPM Trim:**
   ```java
   // Add to driver controls:
   controller.povUp().onTrue(Commands.runOnce(() ->
       shotCalculator.adjustOffset(25)));  // +25 RPM

   controller.povDown().onTrue(Commands.runOnce(() ->
       shotCalculator.adjustOffset(-25))); // -25 RPM
   ```

2. **Reset Between Matches:**
   ```java
   // In Robot.java autonomousInit():
   if (semiAutoHelper != null) {
     shotCalculator.resetOffset();
   }
   ```

### 5.2 Data Logging

**Log these for post-match analysis:**
- All SOTM telemetry
- Actual shot outcomes (made/missed)
- Robot velocity when shooting
- Confidence scores

### 5.3 Performance Metrics

**Success criteria:**
- Confidence >70: shoot confidently
- Confidence 50-70: shoot if desperate
- Confidence <50: don't shoot

---

## 🐛 Troubleshooting Guide

### Problem: Confidence Always Zero

**Causes:**
- Out of range (check `SOTM/DistanceToHub`)
- Behind hub (check robot pose)
- Moving too fast (check `SOTM/RobotSpeedMPS`)

**Fix:**
- Adjust min/maxScoringDistance in config
- Check pose estimation accuracy
- Reduce maxSOTMSpeed if needed

### Problem: Solver Never Converges (Iterations >20)

**Causes:**
- Bad initial LUT (wrong measurements)
- Drag coefficient too high/low
- Very high speeds

**Fix:**
- Verify all CAD measurements
- Try sotmDragCoeff = 0 (disable drag)
- Reduce maxSOTMSpeed

### Problem: Shots Consistently Off

**Direction: Always Short**
- Increase slip factor
- Check exit velocity measurement

**Direction: Always Long**
- Decrease slip factor
- Check for wheel slip

**Direction: Sideways When Moving**
- Check launcherOffsetX/Y
- Verify field velocity calculation
- Tune latency compensation

### Problem: Crashes or Exceptions

**Check:**
- Division by zero (distance = 0)
- NaN in pose/velocity
- Null ShotCalculator (CHURRET_ENABLED false?)

---

## 📈 Expected Performance

### Realistic Goals:

**Static Shots:**
- Distance 1-4m: >95% accuracy

**Moving Shots (0.5 m/s):**
- Distance 1-3m: >80% accuracy

**Moving Shots (1.0 m/s):**
- Distance 1-3m: >60% accuracy

**Moving Shots (>1.5 m/s):**
- Experimental, accuracy varies greatly
- Confidence score will be low

### Confidence Score Guide:

- 90-100: Perfect conditions, shoot immediately
- 70-90: Good shot, high confidence
- 50-70: Risky but shootable
- 30-50: Only if desperate
- <30: Don't shoot

---

## 🚀 Future Enhancements

### After Basic SOTM Works:

1. **Vision Integration:**
   - Wire actual vision confidence
   - Use AprilTag detections for hub position
   - Reset warm start on vision loss

2. **Adaptive Slip Factor:**
   - Track actual vs predicted results
   - Auto-tune slip factor during match
   - Account for battery voltage

3. **Angular Velocity Feedforward:**
   - Use `shot.driveAngularVelocityRadPerSec()`
   - Feed to drive heading controller
   - Smoother tracking during turns

4. **Multi-Target Support:**
   - Switch between hub and passing
   - Auto-select based on position

5. **Shot Predictor Visualization:**
   - Draw predicted trajectory on Glass
   - Show aim point compensation
   - Display confidence heatmap

---

## 📊 Testing Log Template

```markdown
## SOTM Test Log - [DATE]

### Configuration:
- Slip Factor: ____
- Exit Height: ____ m
- Launch Angle: ____ deg
- Launcher Offset: (x:____, y:____)

### Static Tests:
| Distance | RPM | Made/Total | Notes |
|----------|-----|-----------|-------|
| 1.0m     |     |   /5      |       |
| 2.0m     |     |   /5      |       |
| 3.0m     |     |   /5      |       |
| 4.0m     |     |   /5      |       |

### Moving Tests:
| Speed | Distance | Made/Total | Confidence | Notes |
|-------|----------|-----------|------------|-------|
| 0.5   | 2.0m     |   /5      |            |       |
| 1.0   | 2.0m     |   /5      |            |       |

### Issues Found:
-
-

### Next Steps:
-
-
```

---

## ✅ Final Checklist

Before competition:
```
□ All measurements verified
□ Slip factor tuned (<5% error)
□ Static shots tested: 1-4m range
□ Moving shots tested: 0.5, 1.0 m/s
□ Latency compensation tuned
□ Telemetry working and logged
□ Drivers trained on SOTM controls
□ Backup non-SOTM mode works
□ Emergency disable plan (set CHURRET_ENABLED=false)
□ Documented tuned values in notebook
```

---

Good luck with your SOTM system! 🎯
