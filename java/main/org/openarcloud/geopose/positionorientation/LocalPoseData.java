package org.openarcloud.geopose.positionorientation;

public interface LocalPoseData {
    double[] translation = null;

    float[] getRotationAsFloats();
}
