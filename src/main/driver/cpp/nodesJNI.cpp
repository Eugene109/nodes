#include "jni.h"
#include "com_nodes_jni_nodesJNI.h"
#include "trajectoryMath.h"
#include <glm/glm.hpp>
#include <iostream>

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
    // Check to ensure the JNI version is valid

    JNIEnv* env;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
        return JNI_ERR;

    // In here is also where you store things like class references
    // if they are ever needed

    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {}

JNIEXPORT jint JNICALL Java_com_nodes_jni_nodesJNI_initialize
  (JNIEnv *, jclass) {
  return 0;
}

JNIEXPORT jobject JNICALL Java_com_nodes_jni_nodesJNI_newtonRhapsonSolveAirResistance
  (JNIEnv *env, jclass, jdouble v_x, jdouble v_y, jdouble targetX, jdouble targetY, jdouble targetZ, jdouble shooterAltitude) {
    glm::vec3 targetPos = glm::vec3(targetX, targetY, targetZ);
    TrajectorySolution guessSolution = calcFiringSolution(v_x, v_y, targetPos, shooterAltitude);
    // auto guessSolution1 = TrajectorySolution(glm::vec2(8, 3.14159/3.0));
    TrajectorySolution solution = newtonRhapsonSolveAirResistance(v_x, v_y, targetPos, shooterAltitude, guessSolution);

    jclass classObj = env->FindClass("com/nodes/jni/nodesJNI$TrajectorySolution");
    if (classObj == NULL) {
        return NULL; // Handle error/exception
    }
    jmethodID constructorId = env->GetMethodID(classObj, "<init>", "()V");
    if (constructorId == NULL) {
        return NULL; // Handle error/exception
    }
    jobject resultObject = env->NewObject(classObj, constructorId);
    if (resultObject == NULL) {
        return NULL; // Handle error/exception
    }
    env->SetDoubleField(resultObject, env->GetFieldID(classObj, "shooterVel", "D"), solution.shooterVelocity);
    env->SetDoubleField(resultObject, env->GetFieldID(classObj, "azimuth", "D"), solution.azimuth);
  return resultObject;
}
JNIEXPORT jdouble JNICALL Java_com_nodes_jni_nodesJNI_minDistTrajectory
  (JNIEnv *, jclass, jdouble shooterVel, jdouble azimuth, jdouble v_x, jdouble v_y, jdouble targetX, jdouble targetY, jdouble targetZ, jdouble shooterAltitude) {
    double minDist = glm::distance(f_airResistance_RK4(glm::vec2(shooterVel, azimuth), v_x, v_y, targetZ, shooterAltitude), glm::vec2(targetX, targetY));
    return minDist;
}

JNIEXPORT void JNICALL Java_com_nodes_jni_nodesJNI_configureParameters
  (JNIEnv *, jclass, jdouble Cd_new, jdouble A_new, jdouble m_new, jdouble rho_new) {
    configureParameters(Cd_new, A_new, m_new, rho_new);
}