/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.Int2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.data;

import java.util.Collections;
import java.util.function.Function;
import javax.annotation.Nullable;
import org.mtr.core.data.Data;
import org.mtr.core.data.NameColorDataBase;
import org.mtr.core.data.Route;
import org.mtr.core.data.StationExit;
import org.mtr.core.data.TransportMode;
import org.mtr.core.generated.data.StationSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.legacy.data.DataFixer;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.Int2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class Station
extends StationSchema {
    public final ObjectAVLTreeSet<Station> connectedStations = new ObjectAVLTreeSet();

    public Station(Data data) {
        super(TransportMode.values()[0], data);
    }

    public Station(ReaderBase readerBase, Data data) {
        super(DataFixer.convertStation(readerBase), data);
        this.updateData(readerBase);
    }

    public long getZone1() {
        return this.zone1;
    }

    public long getZone2() {
        return this.zone2;
    }

    public long getZone3() {
        return this.zone3;
    }

    public ObjectArrayList<StationExit> getExits() {
        return this.exits;
    }

    public void setZone1(long zone1) {
        this.zone1 = zone1;
    }

    public void setZone2(long zone2) {
        this.zone2 = zone2;
    }

    public void setZone3(long zone3) {
        this.zone3 = zone3;
    }

    public Object2ObjectAVLTreeMap<Station, Int2ObjectAVLTreeMap<ObjectArrayList<Route>>> getInterchangeStationToColorToRoutesMap(boolean includeConnectingStations) {
        Object2ObjectAVLTreeMap stationToColorToRoutesMap = new Object2ObjectAVLTreeMap();
        this.getInterchangeRoutes(includeConnectingStations, stationToColorToRoutesMap, null, station -> station, route -> route);
        return stationToColorToRoutesMap;
    }

    public Object2ObjectAVLTreeMap<String, Int2ObjectAVLTreeMap<ObjectArrayList<String>>> getInterchangeStationNameToColorToRouteNamesMap(boolean includeConnectingStations) {
        Object2ObjectAVLTreeMap stationToColorToRoutesMap = new Object2ObjectAVLTreeMap();
        this.getInterchangeRoutes(includeConnectingStations, stationToColorToRoutesMap, null, NameColorDataBase::getName, route -> String.format("%s||%s", route.getName().split("\\|\\|")[0], route.getRouteNumber()));
        return stationToColorToRoutesMap;
    }

    public ObjectAVLTreeSet<Route> getOneInterchangeRouteFromEachColor(boolean includeConnectingStations) {
        ObjectAVLTreeSet oneRouteFromEachColor = new ObjectAVLTreeSet();
        this.getInterchangeRoutes(includeConnectingStations, new Object2ObjectAVLTreeMap(), oneRouteFromEachColor, station -> station, route -> route);
        return oneRouteFromEachColor;
    }

    private <T, U extends Comparable<? super U>> void getInterchangeRoutes(boolean includeConnectingStations, Object2ObjectAVLTreeMap<T, Int2ObjectAVLTreeMap<ObjectArrayList<U>>> stationToColorToRoutesMap, @Nullable ObjectAVLTreeSet<U> oneRouteFromEachColor, Function<Station, T> stationMapper, Function<Route, U> routeMapper) {
        ObjectArrayList<Station> stations = new ObjectArrayList();
        if (includeConnectingStations) {
            stations.addAll(this.connectedStations);
            Collections.sort(stations);
        }
        stations.add(0, this);
        stations.forEach(station -> {
            Int2ObjectAVLTreeMap<ObjectArrayList<U>> colorToRouteMap = new Int2ObjectAVLTreeMap<>();
            station.savedRails.forEach(platform -> platform.routes.forEach(route -> {
                if (!route.getHidden()) {
                    colorToRouteMap.computeIfAbsent(route.getColor(), routes -> new ObjectArrayList<>());
                    U newRoute = routeMapper.apply((Route)route);
                    ObjectArrayList<U> newRoutes = colorToRouteMap.get(route.getColor());
                    if (!newRoutes.contains(newRoute)) {
                        newRoutes.add(routeMapper.apply((Route)route));
                    }
                }
            }));
            if (!colorToRouteMap.isEmpty()) {
                colorToRouteMap.forEach((color, routes) -> {
                    Collections.sort(routes);
                    if (oneRouteFromEachColor != null) {
                        oneRouteFromEachColor.add(routes.get(0));
                    }
                });
                stationToColorToRoutesMap.put(stationMapper.apply((Station)station), colorToRouteMap);
            }
        });
    }
}

