/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
 */
package org.mtr.core.data;

import org.mtr.core.data.Platform;
import org.mtr.core.data.Route;
import org.mtr.core.generated.data.RoutePlatformDataSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;

public final class RoutePlatformData
extends RoutePlatformDataSchema {
    public Platform platform;

    public RoutePlatformData(long platformId) {
        super(platformId);
    }

    public RoutePlatformData(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public Platform getPlatform() {
        return this.platform;
    }

    public String getCustomDestination() {
        return this.customDestination;
    }

    public void setCustomDestination(String customDestination) {
        this.customDestination = customDestination;
    }

    public void writePlatformCache(Route route, Long2ObjectOpenHashMap<Platform> platformIdMap) {
        this.platform = (Platform)platformIdMap.get(this.platformId);
        if (this.platform != null) {
            this.platform.routes.add(route);
            this.platform.routeColors.add(route.getColor());
        }
    }
}

