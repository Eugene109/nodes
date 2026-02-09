package com.nodes.jni;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Demo class for loading the driver via JNI.
 */
public class nodesJNI {
  static boolean libraryLoaded = false;

  /**
   * Helper class for determining whether or not to load the driver on static initialization.
   */
  public static class Helper {
    private static AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

    /**
     * Get whether to load the driver on static init.
     * @return true if the driver will load on static init
     */
    public static boolean getExtractOnStaticLoad() {
      return extractOnStaticLoad.get();
    }

    /**
     * Set whether to load the driver on static init.
     * @param load the new value
     */
    public static void setExtractOnStaticLoad(boolean load) {
      extractOnStaticLoad.set(load);
    }
  }

  static {
    if (Helper.getExtractOnStaticLoad()) {
      System.loadLibrary("nodesDriver");
      libraryLoaded = true;
    }
  }

  /**
   * Force load the library.
   */
  public static synchronized void forceLoad() {
    if (libraryLoaded) {
      return;
    }
    System.loadLibrary("nodesDriver");
    libraryLoaded = true;
  }

  /**
   * Tells the driver to initialize.
   * This is a demo of a native JNI method from the driver.
   * 
   * @return the int returned by the driver
   * @see "nodesJNI.cpp"
   */
  public static native int initialize();

  public static class TrajectorySolution {
    public double shooterVel;
    public double azimuth;    
    public TrajectorySolution() {} // No-arg constructor for JNI
    
    public TrajectorySolution(double shooterVel, double azimuth) {
      this.shooterVel = shooterVel;
      this.azimuth = azimuth;
    }
  }
  public static native TrajectorySolution newtonRhapsonSolveAirResistance(
      double v_x,
      double v_y,
      double targetX,
      double targetY,
      double targetZ,
      double shooterAltitude);
}
