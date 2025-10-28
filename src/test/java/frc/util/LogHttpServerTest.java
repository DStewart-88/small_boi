package frc.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;

import java.io.IOException;
import java.net.ConnectException;
import java.net.HttpURLConnection;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.FileTime;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class LogHttpServerTest {
  private LogHttpServer server;
  private Thread serverThread;
  private int port;
  private Path tempDir;

  @BeforeEach
  public void setup() throws Exception {
    // Disabled by default for tests
    setEnabled(false);

    tempDir = Files.createTempDirectory("log-server-test");
    port = findFreePort();
    server = new LogHttpServer(port, tempDir);
    serverThread = new Thread(server, "LogHttpServerTest-main");
    serverThread.setDaemon(true);
    serverThread.start();
    // Small sleep to allow bind
    Thread.sleep(25);
  }

  @AfterEach
  public void teardown() throws Exception {
    if (server != null) server.stop();
    if (serverThread != null) serverThread.join(500);
    if (tempDir != null) {
      try (var s = Files.list(tempDir)) {
        s.forEach(p -> {
          try { Files.deleteIfExists(p); } catch (IOException ignored) {}
        });
      } catch (IOException ignored) {}
      try { Files.deleteIfExists(tempDir); } catch (IOException ignored) {}
    }
  }

  @Test
  public void healthAndStatusWhenDisabled() throws Exception {
    Response r1 = httpGet("/");
    assertEquals(200, r1.code);
    assertTrue(new String(r1.body, StandardCharsets.UTF_8).contains("log server is online"));

    Response r2 = httpGet("/status.json");
    assertEquals(200, r2.code);
    String s = new String(r2.body, StandardCharsets.UTF_8).toLowerCase(Locale.ROOT);
    assertTrue(s.contains("\"enabled\":false"), "Status should report enabled:false when disabled");
  }

  @Test
  public void logsIndexNewestFirstAndFilter() throws Exception {
    Path fOld = tempDir.resolve("a_old.wpilog");
    Path fNew = tempDir.resolve("b_new.hoot");
    Path fSkip = tempDir.resolve("c_skip.txt");
    Files.writeString(fOld, "old");
    Files.writeString(fNew, "newer");
    Files.writeString(fSkip, "skip");

    long now = System.currentTimeMillis();
    Files.setLastModifiedTime(fOld, FileTime.fromMillis(now - 2000));
    Files.setLastModifiedTime(fNew, FileTime.fromMillis(now - 1000));
    Files.setLastModifiedTime(fSkip, FileTime.fromMillis(now - 500));

    Response r = httpGet("/logs.json");
    assertEquals(200, r.code);
    String json = new String(r.body, StandardCharsets.UTF_8);
    // crude parse: ["b_new.hoot","a_old.wpilog"]
    List<String> names = parseJsonArrayOfStrings(json);
    assertEquals(2, names.size());
    assertEquals("b_new.hoot", names.get(0));
    assertEquals("a_old.wpilog", names.get(1));
  }

  @Test
  public void fileDownloadAndLength() throws Exception {
    byte[] content = "0123456789".getBytes(StandardCharsets.UTF_8);
    Path f = tempDir.resolve("test.wpilog");
    Files.write(f, content);
    Files.setLastModifiedTime(f, FileTime.from(Instant.now()));

    Response r = httpGet("/test.wpilog");
    assertEquals(200, r.code);
    assertEquals(content.length, r.contentLength);
    assertArrayEquals(content, r.body);
  }

  @Test
  public void returns503WhenEnabled() throws Exception {
    setEnabled(true);
    Response r = httpGet("/");
    assertEquals(503, r.code);
  }

  @Test
  public void stopClosesPort() throws Exception {
    server.stop();
    serverThread.join(500);

    // Attempt connection should fail quickly
    boolean refused = false;
    try (Socket s = new Socket("127.0.0.1", port)) {
      // If connect succeeds, the server is still listening; fail the test
      refused = false;
    } catch (ConnectException ce) {
      refused = true;
    }
    assertTrue(refused, "Connection should be refused after server stop");
  }

  // ---- Helpers ----
  private static int findFreePort() throws IOException {
    try (ServerSocket s = new ServerSocket(0)) {
      return s.getLocalPort();
    }
  }

  private void setEnabled(boolean en) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(en);
    DriverStationSim.notifyNewData();
  }

  private Response httpGet(String path) throws Exception {
    URL url = new URL("http://127.0.0.1:" + port + path);
    HttpURLConnection conn = (HttpURLConnection) url.openConnection();
    conn.setInstanceFollowRedirects(false);
    conn.setRequestMethod("GET");
    conn.setConnectTimeout(500);
    conn.setReadTimeout(2000);
    int code = conn.getResponseCode();
    long len = conn.getContentLengthLong();
    byte[] body;
    try (var is = (code >= 400) ? conn.getErrorStream() : conn.getInputStream()) {
      if (is != null) {
        body = is.readAllBytes();
      } else {
        body = new byte[0];
      }
    }
    conn.disconnect();
    Response r = new Response();
    r.code = code;
    r.contentLength = (int) (len < 0 ? body.length : len);
    r.body = body;
    return r;
  }

  private static List<String> parseJsonArrayOfStrings(String json) {
    List<String> out = new ArrayList<>();
    String s = json.trim();
    if (s.length() < 2 || s.charAt(0) != '[' || s.charAt(s.length() - 1) != ']') return out;
    s = s.substring(1, s.length() - 1).trim();
    if (s.isEmpty()) return out;
    // Split on commas not inside quotes (simple filenames will not contain commas)
    String[] parts = s.split(",");
    for (String p : parts) {
      String t = p.trim();
      if (t.startsWith("\"") && t.endsWith("\"") && t.length() >= 2) {
        t = t.substring(1, t.length() - 1);
      }
      // unescape minimal (quotes/backslashes not expected in filenames under our regex)
      out.add(t);
    }
    return out;
  }

  private static class Response {
    int code;
    int contentLength;
    byte[] body;
  }
}

