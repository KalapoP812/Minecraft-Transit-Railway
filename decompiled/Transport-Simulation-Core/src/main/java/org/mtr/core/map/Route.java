/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList
 */
package org.mtr.core.map;

import org.mtr.core.generated.map.RouteSchema;
import org.mtr.core.map.RouteStation;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList;

public final class Route
extends RouteSchema {
    public Route(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    Route(org.mtr.core.data.Route route) {
        super(route.getHexId(), route.getName(), route.getColor(), route.getRouteNumber(), route.getRouteTypeKey(), route.getCircularState(), route.getHidden());
        route.getRoutePlatforms().forEach(routePlatformData -> {
            if (routePlatformData.platform != null && routePlatformData.platform.area != null) {
                this.stations.add(RouteStation.create(routePlatformData.platform));
            }
        });
        this.durations.addAll((LongList)route.durations);
        route.depots.forEach(depot -> this.depots.add(depot.getName()));
    }
}

