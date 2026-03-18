/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

public enum RouteType {
    NORMAL,
    LIGHT_RAIL,
    HIGH_SPEED;


    public RouteType next() {
        return RouteType.values()[(this.ordinal() + 1) % RouteType.values().length];
    }
}

