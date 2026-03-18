/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

public enum StopDirection {
    NONE,
    N,
    NE,
    E,
    SE,
    S,
    SW,
    W,
    NW;


    public String toString() {
        return this == NONE ? "" : super.toString();
    }
}

