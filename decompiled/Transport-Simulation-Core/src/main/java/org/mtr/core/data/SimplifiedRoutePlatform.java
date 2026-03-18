/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

import org.mtr.core.generated.data.SimplifiedRoutePlatformSchema;
import org.mtr.core.serializer.ReaderBase;

public final class SimplifiedRoutePlatform
extends SimplifiedRoutePlatformSchema {
    public SimplifiedRoutePlatform(long platformId, long stationId, String destination, String stationName) {
        super(platformId, stationId, destination, stationName);
    }

    public SimplifiedRoutePlatform(ReaderBase readerBase) {
        super(readerBase);
    }

    public long getPlatformId() {
        return this.platformId;
    }

    public long getStationId() {
        return this.stationId;
    }

    @Override
    public String getStationName() {
        return this.stationName;
    }

    public String getDestination() {
        return this.destination;
    }
}

