import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.WebSocket;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Base64;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionStage;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Saves recent robot logs (over SSH) and current NetworkTables values (over NT4) into
 * debug/<timestamp>/. SSH works even when robot code is crash looping; NT4 needs it running.
 */
public final class DownloadRobotDebug {
  private static final Duration RECENT = Duration.ofMinutes(5);
  private static final List<String> LOG_DIRECTORIES =
      List.of("/u/logs", "/U/logs", "/home/lvuser/logs");
  // The robot program's console output, which is where crash stack traces end up.
  private static final List<String> LOG_FILES = List.of("/home/lvuser/FRC_UserProgram.log");
  // Written by the Java runtime itself if it crashes hard (not a normal exception).
  private static final String JAVA_CRASH_FILES = "/home/lvuser/hs_err_pid*.log";
  private static final Duration NT_CONNECT_TIMEOUT = Duration.ofSeconds(3);
  private static final Duration NT_QUIET = Duration.ofSeconds(1);
  private static final Duration NT_MAX_WAIT = Duration.ofSeconds(10);

  private final Path root;
  private final String host;
  private final Path output;
  private final List<String> summary = new ArrayList<>();

  private DownloadRobotDebug(Path root, String host, Path output) {
    this.root = root;
    this.host = host;
    this.output = output;
  }

  public static void main(String[] args) throws Exception {
    Path root = Path.of("").toAbsolutePath();
    String host = args.length > 0 ? args[0] : teamHost(root);
    if (host == null) {
      System.err.println(
          "error: no team number in .wpilib/wpilib_preferences.json; pass the robot address instead");
      System.exit(2);
    }
    String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss"));
    Path output = root.resolve("debug").resolve(timestamp);
    Files.createDirectories(output);

    var download = new DownloadRobotDebug(root, host, output);
    download.note("Robot: " + host);
    download.note("Saved: " + LocalDateTime.now());
    download.downloadLogs();
    download.downloadNetworkTables();
    Files.write(output.resolve("summary.txt"), download.summary);
    System.out.println("\nSaved to " + root.relativize(output));
  }

  private void note(String line) {
    System.out.println(line);
    summary.add(line);
  }

  /** 10.TE.AM.2 from the team number WPILib stores for the project. */
  private static String teamHost(Path root) throws IOException {
    Path preferences = root.resolve(".wpilib/wpilib_preferences.json");
    if (!Files.isRegularFile(preferences)) {
      return null;
    }
    Matcher matcher =
        Pattern.compile("\"teamNumber\"\\s*:\\s*(\\d+)").matcher(Files.readString(preferences));
    if (!matcher.find()) {
      return null;
    }
    int team = Integer.parseInt(matcher.group(1));
    return "10." + team / 100 + "." + team % 100 + ".2";
  }

  // ---------------------------------------------------------------- SSH logs

  private void downloadLogs() {
    System.out.println("\n[ssh] listing logs on " + host + "...");
    // One line with the robot's clock (it may not match this laptop), then "mtime size path".
    // Folders are listed by their real path, since /u and /U can be the same USB stick.
    StringBuilder listing = new StringBuilder("date +%s;");
    for (String directory : LOG_DIRECTORIES) {
      listing.append(" [ -d ").append(directory).append(" ] && (cd ").append(directory)
          .append(" && d=$(pwd -P) && for f in \"$d\"/*; do [ -f \"$f\" ] && stat -c '%Y %s %n' \"$f\"; done);");
    }
    for (String file : LOG_FILES) {
      listing.append(" [ -f ").append(file).append(" ] && stat -c '%Y %s %n' ").append(file).append(";");
    }
    listing.append(" for f in ").append(JAVA_CRASH_FILES)
        .append("; do [ -f \"$f\" ] && stat -c '%Y %s %n' \"$f\"; done;");
    listing.append(" true");

    String text;
    try {
      SshResult result = ssh(listing.toString());
      text = new String(result.stdout(), StandardCharsets.UTF_8);
      if (result.exitCode() != 0) {
        note("Logs: couldn't list files over SSH (exit " + result.exitCode() + "). Is the robot on?");
        return;
      }
    } catch (IOException | InterruptedException e) {
      note("Logs: couldn't run ssh: " + e.getMessage());
      return;
    }

    String[] lines = text.strip().split("\\R");
    long robotNow;
    try {
      robotNow = Long.parseLong(lines[0].strip());
    } catch (NumberFormatException e) {
      note("Logs: unexpected listing from robot:\n" + text);
      return;
    }

    // Keep everything changed in the last few minutes (crash loops), plus the newest log.
    Map<String, long[]> files = new LinkedHashMap<>();
    for (int i = 1; i < lines.length; i++) {
      String[] parts = lines[i].strip().split(" ", 3);
      if (parts.length == 3) {
        files.putIfAbsent(parts[2], new long[] {Long.parseLong(parts[0]), Long.parseLong(parts[1])});
      }
    }
    List<String> selected = new ArrayList<>();
    String newest = null;
    long newestModified = Long.MIN_VALUE;
    for (var entry : files.entrySet()) {
      long modified = entry.getValue()[0];
      if (modified >= robotNow - RECENT.toSeconds()) {
        selected.add(entry.getKey());
      }
      if (!LOG_FILES.contains(entry.getKey()) && !entry.getKey().contains("/hs_err_pid")
          && modified > newestModified) {
        newest = entry.getKey();
        newestModified = modified;
      }
    }
    if (newest != null && !selected.contains(newest)) {
      selected.add(newest);
    }
    for (String file : LOG_FILES) {
      if (files.containsKey(file) && !selected.contains(file)) {
        selected.add(file);
      }
    }
    if (selected.isEmpty()) {
      note("Logs: none found on the robot.");
      return;
    }

    Set<String> usedNames = new HashSet<>();
    for (String remote : selected) {
      String base = remote.substring(remote.lastIndexOf('/') + 1);
      String name = base;
      for (int copy = 2; !usedNames.add(name); copy++) {
        name = copy + "-" + base;
      }
      System.out.println("[ssh] downloading " + remote + " (" + files.get(remote)[1] / 1024 + " KB)");
      try {
        SshResult result = ssh("cat '" + remote.replace("'", "'\\''") + "'");
        if (result.exitCode() != 0) {
          note("Log " + remote + ": failed (exit " + result.exitCode() + ")");
          continue;
        }
        Files.write(output.resolve(name), result.stdout());
        note("Log " + remote + " -> " + name);
        if (name.endsWith(".wpilog")) {
          saveConsoleText(name, result.stdout());
        }
      } catch (IOException | InterruptedException e) {
        note("Log " + remote + ": failed: " + e.getMessage());
      }
    }
  }

  /** Pulls the robot's console output out of a binary .wpilog into a readable .txt next to it. */
  private void saveConsoleText(String logName, byte[] log) {
    String textName = logName.substring(0, logName.length() - ".wpilog".length()) + ".console.txt";
    try {
      List<String> lines = WpiLog.consoleLines(log);
      if (lines.isEmpty()) {
        note("  (no console output in " + logName + ")");
        return;
      }
      Files.write(output.resolve(textName), lines);
      note("  console text -> " + textName);
    } catch (RuntimeException | IOException e) {
      note("  couldn't read console text from " + logName + ": " + e);
    }
  }

  private record SshResult(int exitCode, byte[] stdout) {}

  /** Runs a command on the roboRIO as admin, which has no password. */
  private SshResult ssh(String command) throws IOException, InterruptedException {
    boolean windows = System.getProperty("os.name").toLowerCase().contains("win");
    List<String> args =
        List.of(
            "ssh",
            "-o", "ConnectTimeout=5",
            // The roboRIO's host key changes every time it's reimaged, so don't pin it.
            "-o", "StrictHostKeyChecking=no",
            "-o", "UserKnownHostsFile=" + (windows ? "NUL" : "/dev/null"),
            "-o", "LogLevel=ERROR",
            "admin@" + host,
            command);
    Process process =
        new ProcessBuilder(args)
            .directory(root.toFile())
            .redirectInput(ProcessBuilder.Redirect.INHERIT)
            .redirectError(ProcessBuilder.Redirect.INHERIT)
            .start();
    byte[] stdout;
    try (InputStream in = process.getInputStream()) {
      stdout = in.readAllBytes();
    }
    return new SshResult(process.waitFor(), stdout);
  }

  // ---------------------------------------------------------------- NT4 snapshot

  private void downloadNetworkTables() {
    System.out.println("\n[nt4] connecting to " + host + ":5810...");
    Map<Long, Map<String, Object>> topicsById = new TreeMap<>();
    Map<Long, Object> values = new TreeMap<>();
    long[] lastAnnounce = {System.nanoTime()};
    ByteArrayOutputStream binary = new ByteArrayOutputStream();
    StringBuilder textMessage = new StringBuilder();

    WebSocket.Listener listener =
        new WebSocket.Listener() {
          @Override
          public void onOpen(WebSocket socket) {
            socket.sendText(
                "[{\"method\":\"subscribe\",\"params\":{\"topics\":[\"\"],\"subuid\":1,"
                    + "\"options\":{\"prefix\":true}}}]",
                true);
            socket.request(1);
          }

          @Override
          public CompletionStage<?> onText(WebSocket socket, CharSequence data, boolean last) {
            textMessage.append(data);
            if (last) {
              synchronized (topicsById) {
                for (Object message : (List<?>) Json.parse(textMessage.toString())) {
                  var map = (Map<?, ?>) message;
                  var params = (Map<?, ?>) map.get("params");
                  if ("announce".equals(map.get("method"))) {
                    Map<String, Object> topic = new LinkedHashMap<>();
                    topic.put("type", params.get("type"));
                    topic.put("properties", params.get("properties"));
                    topic.put("name", params.get("name"));
                    topicsById.put(((Number) params.get("id")).longValue(), topic);
                    lastAnnounce[0] = System.nanoTime();
                  }
                }
              }
              textMessage.setLength(0);
            }
            socket.request(1);
            return null;
          }

          @Override
          public CompletionStage<?> onBinary(WebSocket socket, ByteBuffer data, boolean last) {
            byte[] chunk = new byte[data.remaining()];
            data.get(chunk);
            binary.writeBytes(chunk);
            if (last) {
              synchronized (topicsById) {
                MsgPack reader = new MsgPack(binary.toByteArray());
                while (reader.hasMore()) {
                  if (reader.read() instanceof List<?> sample && sample.size() == 4) {
                    long id = ((Number) sample.get(0)).longValue();
                    if (id >= 0) {
                      values.put(id, sample.get(3));
                    }
                  }
                }
              }
              binary.reset();
            }
            socket.request(1);
            return null;
          }
        };

    WebSocket socket;
    try {
      socket =
          HttpClient.newBuilder()
              .connectTimeout(NT_CONNECT_TIMEOUT)
              .build()
              .newWebSocketBuilder()
              .subprotocols("v4.1.networktables.first.wpi.edu", "networktables.first.wpi.edu")
              .connectTimeout(NT_CONNECT_TIMEOUT)
              .buildAsync(URI.create("ws://" + host + ":5810/nt/download-robot-debug"), listener)
              .get(NT_CONNECT_TIMEOUT.toMillis() + 1000, TimeUnit.MILLISECONDS);
    } catch (Exception e) {
      note("NetworkTables: couldn't connect (is robot code running?): " + rootMessage(e));
      return;
    }

    // Values for every topic arrive right after the announcements, so wait for those to settle.
    long start = System.nanoTime();
    lastAnnounce[0] = start;
    try {
      while (System.nanoTime() - start < NT_MAX_WAIT.toNanos()
          && System.nanoTime() - lastAnnounce[0] < NT_QUIET.toNanos()) {
        Thread.sleep(100);
      }
      Thread.sleep(300);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
    socket.sendClose(WebSocket.NORMAL_CLOSURE, "done");

    Map<String, Object> snapshot = new TreeMap<>();
    synchronized (topicsById) {
      for (var entry : topicsById.entrySet()) {
        Map<String, Object> topic = new LinkedHashMap<>(entry.getValue());
        String name = (String) topic.remove("name");
        Object value = values.get(entry.getKey());
        topic.put("value", value instanceof byte[] bytes
            ? Map.of("base64", Base64.getEncoder().encodeToString(bytes))
            : value);
        snapshot.put(name, topic);
      }
    }
    try {
      Files.writeString(output.resolve("networktables.json"), Json.write(snapshot, 0) + "\n");
      note("NetworkTables: saved " + snapshot.size() + " topics -> networktables.json");
    } catch (IOException e) {
      note("NetworkTables: couldn't save: " + e.getMessage());
    }
  }

  private static String rootMessage(Throwable error) {
    while (error.getCause() != null) {
      error = error.getCause();
    }
    return error.toString();
  }

  /** Just enough MessagePack to read NT4 value frames. */
  private static final class MsgPack {
    private final ByteBuffer in;

    MsgPack(byte[] data) {
      in = ByteBuffer.wrap(data);
    }

    boolean hasMore() {
      return in.hasRemaining();
    }

    Object read() {
      int tag = in.get() & 0xff;
      if (tag <= 0x7f) return (long) tag;
      if (tag >= 0xe0) return (long) (tag - 256);
      if ((tag & 0xe0) == 0xa0) return string(tag & 0x1f);
      if ((tag & 0xf0) == 0x90) return array(tag & 0x0f);
      if ((tag & 0xf0) == 0x80) return map(tag & 0x0f);
      return switch (tag) {
        case 0xc0 -> null;
        case 0xc2 -> false;
        case 0xc3 -> true;
        case 0xc4 -> bytes(in.get() & 0xff);
        case 0xc5 -> bytes(in.getShort() & 0xffff);
        case 0xc6 -> bytes(in.getInt());
        case 0xca -> (double) in.getFloat();
        case 0xcb -> in.getDouble();
        case 0xcc -> (long) (in.get() & 0xff);
        case 0xcd -> (long) (in.getShort() & 0xffff);
        case 0xce -> in.getInt() & 0xffffffffL;
        case 0xcf, 0xd3 -> in.getLong();
        case 0xd0 -> (long) in.get();
        case 0xd1 -> (long) in.getShort();
        case 0xd2 -> (long) in.getInt();
        case 0xd9 -> string(in.get() & 0xff);
        case 0xda -> string(in.getShort() & 0xffff);
        case 0xdb -> string(in.getInt());
        case 0xdc -> array(in.getShort() & 0xffff);
        case 0xdd -> array(in.getInt());
        case 0xde -> map(in.getShort() & 0xffff);
        case 0xdf -> map(in.getInt());
        default -> throw new IllegalStateException("Unsupported MessagePack tag " + tag);
      };
    }

    private byte[] bytes(int length) {
      byte[] data = new byte[length];
      in.get(data);
      return data;
    }

    private String string(int length) {
      return new String(bytes(length), StandardCharsets.UTF_8);
    }

    private List<Object> array(int length) {
      List<Object> list = new ArrayList<>(length);
      for (int i = 0; i < length; i++) list.add(read());
      return list;
    }

    private Map<Object, Object> map(int length) {
      Map<Object, Object> map = new LinkedHashMap<>();
      for (int i = 0; i < length; i++) map.put(read(), read());
      return map;
    }
  }

  /** Just enough JSON to read NT4 announcements and write the snapshot. */
  private static final class Json {
    private final String text;
    private int at;

    private Json(String text) {
      this.text = text;
    }

    static Object parse(String text) {
      return new Json(text).value();
    }

    private Object value() {
      skipSpace();
      char c = text.charAt(at);
      if (c == '{') {
        Map<String, Object> map = new LinkedHashMap<>();
        at++;
        skipSpace();
        if (text.charAt(at) == '}') {
          at++;
          return map;
        }
        while (true) {
          skipSpace();
          String key = string();
          skipSpace();
          at++; // :
          map.put(key, value());
          skipSpace();
          if (text.charAt(at++) == '}') return map;
        }
      }
      if (c == '[') {
        List<Object> list = new ArrayList<>();
        at++;
        skipSpace();
        if (text.charAt(at) == ']') {
          at++;
          return list;
        }
        while (true) {
          list.add(value());
          skipSpace();
          if (text.charAt(at++) == ']') return list;
        }
      }
      if (c == '"') return string();
      if (text.startsWith("true", at)) {
        at += 4;
        return true;
      }
      if (text.startsWith("false", at)) {
        at += 5;
        return false;
      }
      if (text.startsWith("null", at)) {
        at += 4;
        return null;
      }
      int start = at;
      while (at < text.length() && "+-0123456789.eE".indexOf(text.charAt(at)) >= 0) at++;
      String number = text.substring(start, at);
      return number.matches("-?\\d+") ? (Object) Long.parseLong(number) : Double.parseDouble(number);
    }

    private String string() {
      StringBuilder out = new StringBuilder();
      at++; // opening quote
      while (true) {
        char c = text.charAt(at++);
        if (c == '"') return out.toString();
        if (c != '\\') {
          out.append(c);
          continue;
        }
        char escaped = text.charAt(at++);
        switch (escaped) {
          case 'n' -> out.append('\n');
          case 't' -> out.append('\t');
          case 'r' -> out.append('\r');
          case 'b' -> out.append('\b');
          case 'f' -> out.append('\f');
          case 'u' -> {
            out.append((char) Integer.parseInt(text.substring(at, at + 4), 16));
            at += 4;
          }
          default -> out.append(escaped);
        }
      }
    }

    private void skipSpace() {
      while (at < text.length() && Character.isWhitespace(text.charAt(at))) at++;
    }

    static String write(Object value, int indent) {
      String pad = "  ".repeat(indent + 1);
      String close = "  ".repeat(indent);
      if (value == null) return "null";
      if (value instanceof String string) return quote(string);
      if (value instanceof Double number && !Double.isFinite(number)) return quote(number.toString());
      if (value instanceof Number || value instanceof Boolean) return value.toString();
      if (value instanceof Map<?, ?> map) {
        if (map.isEmpty()) return "{}";
        List<String> items = new ArrayList<>();
        for (var entry : map.entrySet()) {
          items.add(pad + quote(String.valueOf(entry.getKey())) + ": " + write(entry.getValue(), indent + 1));
        }
        return "{\n" + String.join(",\n", items) + "\n" + close + "}";
      }
      if (value instanceof List<?> list) {
        List<String> items = new ArrayList<>();
        for (Object item : list) items.add(write(item, indent + 1));
        return "[" + String.join(", ", items) + "]";
      }
      return quote(value.toString());
    }

    private static String quote(String string) {
      StringBuilder out = new StringBuilder("\"");
      for (char c : string.toCharArray()) {
        switch (c) {
          case '"' -> out.append("\\\"");
          case '\\' -> out.append("\\\\");
          case '\n' -> out.append("\\n");
          case '\r' -> out.append("\\r");
          case '\t' -> out.append("\\t");
          default -> {
            if (c < 0x20) out.append(String.format("\\u%04x", (int) c));
            else out.append(c);
          }
        }
      }
      return out.append('"').toString();
    }
  }

  /**
   * Just enough of the WPILOG format to read DataLogManager's "console" and "messages" text.
   * See https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/doc/datalog.adoc
   */
  private static final class WpiLog {
    private static final Set<String> TEXT_ENTRIES = Set.of("console", "messages");

    static List<String> consoleLines(byte[] data) {
      ByteBuffer in = ByteBuffer.wrap(data).order(java.nio.ByteOrder.LITTLE_ENDIAN);
      if (data.length < 12 || !new String(data, 0, 6, StandardCharsets.US_ASCII).equals("WPILOG")) {
        throw new IllegalArgumentException("not a WPILOG file");
      }
      in.position(8);
      in.position(12 + in.getInt());

      Map<Long, String> textEntryNames = new HashMap<>();
      List<String> lines = new ArrayList<>();
      // The newest run's file may still be open, so stop quietly at a cut-off record.
      while (in.remaining() >= 1) {
        int header = in.get() & 0xff;
        int idLength = (header & 0x3) + 1;
        int sizeLength = ((header >> 2) & 0x3) + 1;
        int timestampLength = ((header >> 4) & 0x7) + 1;
        if (in.remaining() < idLength + sizeLength + timestampLength) break;
        long id = readUnsigned(in, idLength);
        int size = (int) readUnsigned(in, sizeLength);
        long timestampMicros = readUnsigned(in, timestampLength);
        if (size < 0 || in.remaining() < size) break;
        byte[] payload = new byte[size];
        in.get(payload);

        if (id == 0) {
          // Control record: type 0 starts an entry (id, name, type, metadata).
          ByteBuffer control = ByteBuffer.wrap(payload).order(java.nio.ByteOrder.LITTLE_ENDIAN);
          if (control.remaining() >= 5 && control.get() == 0) {
            long entryId = control.getInt() & 0xffffffffL;
            String name = readString(control);
            String type = readString(control);
            if (TEXT_ENTRIES.contains(name) && type.equals("string")) {
              textEntryNames.put(entryId, name);
            }
          }
        } else if (textEntryNames.containsKey(id)) {
          String prefix = String.format("[%9.3fs]%s ", timestampMicros / 1e6,
              textEntryNames.get(id).equals("messages") ? " (message)" : "");
          String text = new String(payload, StandardCharsets.UTF_8);
          for (String line : text.split("\\R")) {
            if (!line.isEmpty()) lines.add(prefix + line);
          }
        }
      }
      return lines;
    }

    private static long readUnsigned(ByteBuffer in, int length) {
      long value = 0;
      for (int i = 0; i < length; i++) value |= (long) (in.get() & 0xff) << (8 * i);
      return value;
    }

    private static String readString(ByteBuffer in) {
      byte[] bytes = new byte[in.getInt()];
      in.get(bytes);
      return new String(bytes, StandardCharsets.UTF_8);
    }
  }
}
