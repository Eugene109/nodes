package com.nodes.jni;

import org.junit.jupiter.api.Test;

public class nodesJNITest {
  @Test
  void jniLinkTest() {
    // Test to verify that the JNI test link works correctly.
    nodesJNI.initialize();
  }
  @Test
  void jniDoubleTest() {
    // Test to verify that the JNI test link works correctly.
    nodesJNI.newtonRhapsonSolveAirResistance(0.0, 0.0, 5.0, 5.0, 1.81, 3.14159/3.0);
  }
}