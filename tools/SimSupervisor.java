import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.BasicFileAttributes;
import java.time.Duration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Stream;

/** Runs the newest robot simulation, restarting after relevant project changes. */
public final class SimSupervisor {
  private static final Duration POLL_INTERVAL = Duration.ofMillis(150);
  private static final Duration DEBOUNCE = Duration.ofMillis(350);
  private static final Duration CRASH_RESTART_DELAY = Duration.ofSeconds(1);
  private static final Duration MAX_FAILURE_RESTART_DELAY = Duration.ofSeconds(10);
  private static final List<String> WATCH_DIRECTORIES =
      List.of("src/main/java", "src/main/deploy", "vendordeps");
  private static final List<String> WATCH_FILES =
      List.of(
          "build.gradle",
          "settings.gradle",
          "gradle.properties",
          "gradle/wrapper/gradle-wrapper.properties");

  private record FileState(long modifiedMillis, long size) {}

  private final Path root;
  private Process simulator;
  private long restartAtNanos;
  private int consecutiveFailedExits;

  private SimSupervisor(Path root) {
    this.root = root;
  }

  public static void main(String[] args) throws Exception {
    Path root =
        args.length == 0 ? Path.of("").toAbsolutePath() : Path.of(args[0]).toAbsolutePath();
    if (!Files.isRegularFile(root.resolve("build.gradle"))
        || !Files.isRegularFile(root.resolve(gradleWrapper()))) {
      System.err.println("error: " + root + " is not a WPILib Gradle project");
      System.exit(2);
    }

    var supervisor = new SimSupervisor(root);
    Runtime.getRuntime().addShutdownHook(new Thread(supervisor::stop));
    supervisor.run();
  }

  private void run() throws Exception {
    System.out.println("[watch] project: " + root);
    System.out.println("[watch] Java: " + System.getProperty("java.home"));
    System.out.println("[watch] press Ctrl-C to stop");
    Map<Path, FileState> current = snapshot();
    Set<Path> pending = new TreeSet<>();
    long lastChange = 0;
    start();

    while (!Thread.currentThread().isInterrupted()) {
      Thread.sleep(POLL_INTERVAL.toMillis());
      Map<Path, FileState> updated = snapshot();
      for (Path path : union(current.keySet(), updated.keySet())) {
        if (!Objects.equals(current.get(path), updated.get(path))) {
          pending.add(path);
          lastChange = System.nanoTime();
        }
      }
      current = updated;

      if (!pending.isEmpty()
          && System.nanoTime() - lastChange >= DEBOUNCE.toNanos()) {
        System.out.println("\n[watch] changed: " + String.join(", ", pathsToStrings(pending)));
        pending.clear();
        restart();
      } else if (simulator != null && !simulator.isAlive()) {
        int exitCode = simulator.exitValue();
        simulator = null;
        long delayMillis = restartDelayMillis(exitCode);
        restartAtNanos = System.nanoTime() + Duration.ofMillis(delayMillis).toNanos();
        System.out.println(
            "[sim] exited unexpectedly with code "
                + exitCode
                + "; restarting in "
                + delayMillis
                + " ms");
      } else if (simulator == null
          && restartAtNanos != 0
          && System.nanoTime() >= restartAtNanos) {
        restartAtNanos = 0;
        start();
      }
    }
  }

  private Map<Path, FileState> snapshot() throws IOException {
    Map<Path, FileState> result = new HashMap<>();
    for (String directoryName : WATCH_DIRECTORIES) {
      Path directory = root.resolve(directoryName);
      if (!Files.isDirectory(directory)) {
        continue;
      }
      try (Stream<Path> paths = Files.walk(directory)) {
        for (Path path : paths.filter(Files::isRegularFile).toList()) {
          if (!path.getFileName().toString().equals("BuildConstants.java")
              && !path.getFileName().toString().equals(".DS_Store")) {
            addState(result, path);
          }
        }
      }
    }
    for (String fileName : WATCH_FILES) {
      Path path = root.resolve(fileName);
      if (Files.isRegularFile(path)) {
        addState(result, path);
      }
    }
    return result;
  }

  private void addState(Map<Path, FileState> result, Path path) throws IOException {
    BasicFileAttributes attributes = Files.readAttributes(path, BasicFileAttributes.class);
    result.put(
        root.relativize(path),
        new FileState(attributes.lastModifiedTime().toMillis(), attributes.size()));
  }

  private void start() throws IOException {
    List<String> command =
        List.of(
            root.resolve(gradleWrapper()).toString(),
            "--no-daemon",
            "simulateJava",
            "--console=plain");
    System.out.println("\n[sim] starting: " + String.join(" ", command));
    ProcessBuilder builder = new ProcessBuilder(command).directory(root.toFile()).inheritIO();
    String javaHome = System.getProperty("java.home");
    String currentPath = builder.environment().getOrDefault("PATH", "");
    builder.environment().put("JAVA_HOME", javaHome);
    builder
        .environment()
        .put("PATH", Path.of(javaHome, "bin") + java.io.File.pathSeparator + currentPath);
    simulator = builder.start();
  }

  private long restartDelayMillis(int exitCode) {
    if (exitCode == 0) {
      consecutiveFailedExits = 0;
      return CRASH_RESTART_DELAY.toMillis();
    }
    consecutiveFailedExits = Math.min(consecutiveFailedExits + 1, 10);
    long multiplier = 1L << Math.min(consecutiveFailedExits - 1, 4);
    return Math.min(
        CRASH_RESTART_DELAY.toMillis() * multiplier,
        MAX_FAILURE_RESTART_DELAY.toMillis());
  }

  private void restart() throws Exception {
    stop();
    restartAtNanos = 0;
    consecutiveFailedExits = 0;
    start();
  }

  private synchronized void stop() {
    Process process = simulator;
    simulator = null;
    if (process == null || !process.isAlive()) {
      return;
    }
    System.out.println("[sim] stopping current simulator...");
    List<ProcessHandle> descendants = process.descendants().toList();
    destroyDescendants(descendants, false);
    process.destroy();
    try {
      if (!process.waitFor(5, java.util.concurrent.TimeUnit.SECONDS)) {
        destroyDescendants(descendants, true);
        process.destroyForcibly().waitFor();
      }
    } catch (InterruptedException exception) {
      Thread.currentThread().interrupt();
      destroyDescendants(descendants, true);
      process.destroyForcibly();
    }
  }

  private static void destroyDescendants(List<ProcessHandle> descendants, boolean forcibly) {
    for (int index = descendants.size() - 1; index >= 0; index--) {
      if (forcibly) {
        descendants.get(index).destroyForcibly();
      } else {
        descendants.get(index).destroy();
      }
    }
  }

  private static String gradleWrapper() {
    return System.getProperty("os.name").toLowerCase().contains("win")
        ? "gradlew.bat"
        : "gradlew";
  }

  private static Set<Path> union(Set<Path> first, Set<Path> second) {
    Set<Path> result = new TreeSet<>(first);
    result.addAll(second);
    return result;
  }

  private static List<String> pathsToStrings(Set<Path> paths) {
    return paths.stream().map(Path::toString).toList();
  }
}
