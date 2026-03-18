/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.tool;

public interface EnumHelper {
    public static <T extends Enum<T>> T valueOf(T defaultValue, String name) {
        try {
            return Enum.valueOf(defaultValue.getDeclaringClass(), name);
        }
        catch (Exception e) {
            return defaultValue;
        }
    }
}

