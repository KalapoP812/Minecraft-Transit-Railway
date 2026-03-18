/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.map;

import java.util.Arrays;
import org.mtr.core.generated.map.StationsAndRoutesSchema;
import org.mtr.core.map.Route;
import org.mtr.core.map.Station;

public final class StationAndRoutes
extends StationsAndRoutesSchema {
    public StationAndRoutes(String[] dimensions) {
        this.dimensions.addAll(Arrays.asList(dimensions));
    }

    public void addStation(org.mtr.core.data.Station station) {
        this.stations.add(new Station(station));
    }

    public void addRoute(org.mtr.core.data.Route route) {
        this.routes.add(new Route(route));
    }
}

