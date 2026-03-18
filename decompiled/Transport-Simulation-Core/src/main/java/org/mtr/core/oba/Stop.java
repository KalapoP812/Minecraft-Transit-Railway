/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import org.mtr.core.generated.oba.StopSchema;
import org.mtr.core.oba.StopDirection;
import org.mtr.core.oba.WheelchairBoarding;
import org.mtr.core.serializer.ReaderBase;

public final class Stop
extends StopSchema {
    public Stop(String id, String code, String name, double lat, double lon, StopDirection direction) {
        super(id, code, name, "", lat, lon, "", 0L, WheelchairBoarding.UNKNOWN, direction);
    }

    public Stop(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void addRouteId(String routeId) {
        this.routeIds.add(routeId);
    }
}

