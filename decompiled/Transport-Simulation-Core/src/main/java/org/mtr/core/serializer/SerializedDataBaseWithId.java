/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.serializer;

import org.mtr.core.serializer.SerializedDataBase;

public interface SerializedDataBaseWithId
extends SerializedDataBase {
    public String getHexId();

    public boolean isValid();
}

