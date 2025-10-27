/*
 This Datalogger class is provided for FTC OnBot Java (OBJ) programmers.
 Most users will not need to edit this class; its methods are called
 from a user's OpMode such as ConceptDatalogger.java or a revised version.
 Credit to @Windwoes (https://github.com/Windwoes).
*/

package org.firstinspires.ftc.teamcode.Utility;

import android.os.Environment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Objects;
import java.util.Arrays; // Used for String.join

/**
 * A simple, minimal Datalogger class for writing CSV files.
 * This version sacrifices type-safety and automatic field management for simplicity,
 * requiring the user to supply all data as String arrays for each log line.
 *
 * IMPORTANT: The user MUST call the close() method when the OpMode stops.
 */
public final class Datalogger implements AutoCloseable {
  private final BufferedCsvWriter writer;
  private final String[] headers;

  /**
   * Constructs the Datalogger, initializes the file, and writes the CSV header.
   * @param filename The base name for the log file (e.g., "my_test").
   * @param headers An array of column names for the CSV file.
   * @throws RuntimeException if the file cannot be created or headers are empty.
   */
  public Datalogger(String filename, String... headers) {
    if (filename == null || filename.isEmpty()) {
      throw new RuntimeException("Filename cannot be empty.");
    }
    if (headers == null || headers.length == 0) {
      throw new RuntimeException("Headers array cannot be empty.");
    }

    this.headers = headers;
    try {
      // Standard FTC log location
      String filepath = String.format(Environment.getExternalStorageDirectory().toString() + "/FIRST/Datalogs/%s.csv", filename);
      this.writer = new BufferedCsvWriter(filepath);
    } catch (IOException e) {
      // Replaced e.printStackTrace() with System.err.println for more controlled logging
      System.err.println("DATALOGGER EXCEPTION: " + e.getMessage());
      throw new RuntimeException("Unable to create output file handle: " + e.getMessage());
    }

    // Write the header line immediately
    try {
      // Arrays.asList(headers).stream().collect(Collectors.joining(",")); // Java 8 not always available
      writer.writeLine(String.join(",", headers));
    } catch (IOException e) {
      throw new RuntimeException("Unable to write header to file :(", e);
    }
  }

  /**
   * Logs a new line of data to the CSV file.
   * The number of values MUST match the number of headers provided in the constructor.
   * All values must be pre-formatted as Strings by the caller.
   * @param values An array of String values corresponding to the headers.
   */
  public void log(String... values) {
    if (values.length != headers.length) {
      // Print error to console but continue operation
      System.err.println("DATALOGGER ERROR: Value count (" + values.length + ") does not match header count (" + headers.length + ")! Line skipped.");
      return;
    }

    try {
      writer.writeLine(String.join(",", values));
    } catch (IOException e) {
      // Replaced e.printStackTrace() with System.err.println for more controlled logging
      System.err.println("DATALOGGER EXCEPTION: " + e.getMessage());
    }
  }

  /**
   * Flushes the buffer and closes the underlying file writer.
   * CRITICAL: Must be called when the OpMode stops to ensure all data is saved.
   */
  @Override
  public void close() {
    try {
      writer.close();
    } catch (IOException e) {
      // Replaced e.printStackTrace() with System.err.println for more controlled logging
      System.err.println("DATALOGGER EXCEPTION: " + e.getMessage());
    }
  }

  // --- CSV Writer Implementation (Nested Class) ---

  private static class BufferedCsvWriter {
    private final BufferedWriter bufferedWriter;

    public BufferedCsvWriter(String filepath) throws IOException {
      File tmp = new File(filepath);
      if (!tmp.exists()) {
        // Ensure parent directories exist before creating the file
        Objects.requireNonNull(tmp.getParentFile()).mkdirs();
      }

      // 'false' means overwrite if file exists (start fresh log)
      FileWriter fileWriter = new FileWriter(filepath, false);
      bufferedWriter = new BufferedWriter(fileWriter);
    }

    public void writeLine(String line) throws IOException {
      bufferedWriter.write(line);
      bufferedWriter.newLine();
    }

    public void close() throws IOException {
      // Flush before closing to ensure everything is written
      bufferedWriter.flush();
      bufferedWriter.close();
    }
  }
}
