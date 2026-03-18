/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectOpenHashSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet
 */
package org.mtr.core.data;

import java.util.Collection;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;
import org.mtr.core.Main;
import org.mtr.core.data.AreaBase;
import org.mtr.core.data.Depot;
import org.mtr.core.data.Lift;
import org.mtr.core.data.NameColorDataBase;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.Route;
import org.mtr.core.data.SavedRailBase;
import org.mtr.core.data.Siding;
import org.mtr.core.data.Station;
import org.mtr.core.map.UpdateDynmap;
import org.mtr.core.map.UpdateSquaremap;
import org.mtr.core.serializer.SerializedDataBaseWithId;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectOpenHashSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet;

public abstract class Data {
    private long currentMillis;
    public final ObjectArraySet<Station> stations = new ObjectArraySet();
    public final ObjectArraySet<Platform> platforms = new ObjectArraySet();
    public final ObjectArraySet<Siding> sidings = new ObjectArraySet();
    public final ObjectArraySet<Route> routes = new ObjectArraySet();
    public final ObjectArraySet<Depot> depots = new ObjectArraySet();
    public final ObjectArraySet<Lift> lifts = new ObjectArraySet();
    public final ObjectArraySet<Rail> rails = new ObjectArraySet();
    public final Long2ObjectOpenHashMap<Station> stationIdMap = new Long2ObjectOpenHashMap();
    public final Long2ObjectOpenHashMap<Platform> platformIdMap = new Long2ObjectOpenHashMap();
    public final Long2ObjectOpenHashMap<Siding> sidingIdMap = new Long2ObjectOpenHashMap();
    public final Long2ObjectOpenHashMap<Route> routeIdMap = new Long2ObjectOpenHashMap();
    public final Long2ObjectOpenHashMap<Depot> depotIdMap = new Long2ObjectOpenHashMap();
    public final Long2ObjectOpenHashMap<Lift> liftIdMap = new Long2ObjectOpenHashMap();
    public final Object2ObjectOpenHashMap<String, Rail> railIdMap = new Object2ObjectOpenHashMap();
    public final Object2ObjectOpenHashMap<Position, Object2ObjectOpenHashMap<Position, Rail>> positionsToRail = new Object2ObjectOpenHashMap();
    public final Object2ObjectOpenHashMap<Position, Rail> runwaysInbound = new Object2ObjectOpenHashMap();
    public final ObjectOpenHashSet<Position> runwaysOutbound = new ObjectOpenHashSet();
    public final Long2ObjectOpenHashMap<Position> platformIdToPosition = new Long2ObjectOpenHashMap();

    public void sync() {
        block9: {
            try {
                this.positionsToRail.clear();
                this.rails.forEach(rail -> rail.writePositionsToRailCache(this.positionsToRail));
                this.rails.forEach(rail -> rail.writeConnectedRailsCacheFromMap(this.positionsToRail));
                this.runwaysInbound.clear();
                this.runwaysOutbound.clear();
                this.rails.forEach(rail -> {
                    if (rail.canConnectRemotely()) {
                        Position position1 = rail.getPosition1();
                        Position position2 = rail.getPosition2();
                        if (rail.speedLimit1MetersPerMillisecond > 0.0) {
                            if (((Object2ObjectOpenHashMap)this.positionsToRail.get(position1)).size() == 1) {
                                this.runwaysInbound.put(position1, rail);
                            }
                            if (((Object2ObjectOpenHashMap)this.positionsToRail.get(position2)).size() == 1) {
                                this.runwaysOutbound.add(position2);
                            }
                        }
                        if (rail.speedLimit2MetersPerMillisecond > 0.0) {
                            if (((Object2ObjectOpenHashMap)this.positionsToRail.get(position2)).size() == 1) {
                                this.runwaysInbound.put(position2, rail);
                            }
                            if (((Object2ObjectOpenHashMap)this.positionsToRail.get(position1)).size() == 1) {
                                this.runwaysOutbound.add(position1);
                            }
                        }
                    }
                });
                if (this instanceof Simulator) {
                    this.platforms.removeIf(platform -> platform.isInvalidSavedRail(this));
                    this.sidings.removeIf(siding -> siding.isInvalidSavedRail(this));
                }
                Data.mapIds(this.stationIdMap, this.stations);
                Data.mapIds(this.platformIdMap, this.platforms);
                Data.mapIds(this.sidingIdMap, this.sidings);
                Data.mapIds(this.routeIdMap, this.routes);
                Data.mapIds(this.depotIdMap, this.depots);
                Data.mapIds(this.liftIdMap, this.lifts);
                Data.mapIds(this.railIdMap, this.rails);
                Data.mapAreasAndSavedRails(this.platforms, this.stations);
                Data.mapAreasAndSavedRails(this.sidings, this.depots);
                this.platformIdToPosition.clear();
                this.platforms.forEach(platform -> {
                    platform.routes.clear();
                    platform.routeColors.clear();
                    this.platformIdToPosition.put(platform.getId(), platform.getMidPosition());
                });
                this.routes.forEach(route -> {
                    route.depots.clear();
                    route.getRoutePlatforms().forEach(routePlatformData -> routePlatformData.writePlatformCache((Route)route, this.platformIdMap));
                    route.getRoutePlatforms().removeIf(routePlatformData -> routePlatformData.platform == null);
                });
                this.depots.forEach(depot -> {
                    depot.writeRouteCache(this.routeIdMap);
                    depot.writePathCache();
                });
                this.stations.forEach(station1 -> {
                    station1.connectedStations.clear();
                    this.stations.forEach(station2 -> {
                        if (station1 != station2 && station1.intersecting(station2)) {
                            station1.connectedStations.add(station2);
                        }
                    });
                });
                if (!(this instanceof Simulator)) break block9;
                try {
                    UpdateSquaremap.updateSquaremap((Simulator)this);
                }
                catch (NoClassDefFoundError noClassDefFoundError) {
                }
                catch (Exception e) {
                    Main.LOGGER.error("", (Throwable)e);
                }
                try {
                    UpdateDynmap.updateDynmap((Simulator)this);
                }
                catch (NoClassDefFoundError e) {
                }
                catch (Exception e) {
                    Main.LOGGER.error("", (Throwable)e);
                }
            }
            catch (Exception e) {
                Main.LOGGER.error("", (Throwable)e);
            }
        }
    }

    public final long getCurrentMillis() {
        return this.currentMillis;
    }

    protected final void setCurrentMillis(long currentMillis) {
        this.currentMillis = currentMillis;
    }

    public static <T, U, V, W extends Map<T, X>, X extends Map<U, V>> V tryGet(W map, T key1, U key2, V defaultValue) {
        V result = Data.tryGet(map, key1, key2);
        return result == null ? defaultValue : result;
    }

    public static <T, U, V, W extends Map<T, X>, X extends Map<U, V>> V tryGet(W map, T key1, U key2) {
        X innerMap = map.get(key1);
        if (innerMap == null) {
            return null;
        }
        return innerMap.get(key2);
    }

    public static <T, U, V extends Map<T, W>, W extends Collection<U>> void put(V map, T key, U newValue, Supplier<W> innerSetSupplier) {
        W newInnerSet;
        W innerSet = map.get(key);
        if (innerSet == null) {
            newInnerSet = innerSetSupplier.get();
            map.put(key, newInnerSet);
        } else {
            newInnerSet = innerSet;
        }
        newInnerSet.add(newValue);
    }

    public static <T, U, V extends Map<T, W>, W extends Collection<U>, X extends Collection<U>> void put(V map, T key, X newValue, Supplier<W> innerSetSupplier) {
        W newInnerSet;
        W innerSet = map.get(key);
        if (innerSet == null) {
            newInnerSet = innerSetSupplier.get();
            map.put(key, newInnerSet);
        } else {
            newInnerSet = innerSet;
        }
        newInnerSet.addAll(newValue);
    }

    public static <T, U, V, W extends Map<T, X>, X extends Map<U, V>> void put(W map, T key1, U key2, Function<V, V> putValue, Supplier<X> innerMapSupplier) {
        X newInnerMap;
        X innerMap = map.get(key1);
        if (innerMap == null) {
            newInnerMap = innerMapSupplier.get();
            map.put(key1, newInnerMap);
        } else {
            newInnerMap = innerMap;
        }
        newInnerMap.put(key2, putValue.apply(newInnerMap.get(key2)));
    }

    private static <U extends NameColorDataBase> void mapIds(Long2ObjectMap<U> map, ObjectSet<U> source) {
        map.clear();
        source.forEach(data -> map.put(data.getId(), data));
    }

    private static <U extends SerializedDataBaseWithId> void mapIds(Object2ObjectMap<String, U> map, ObjectSet<U> source) {
        map.clear();
        source.forEach(data -> map.put(data.getHexId(), data));
    }

    private static <U extends SavedRailBase<U, V>, V extends AreaBase<V, U>> void mapAreasAndSavedRails(ObjectArraySet<U> savedRails, ObjectArraySet<V> areas) {
        areas.forEach(area -> area.savedRails.clear());
        savedRails.forEach(savedRail -> {
            savedRail.area = null;
            Position pos = savedRail.getMidPosition();
            for (AreaBase area : areas) {
                if (!area.isTransportMode((NameColorDataBase)savedRail) || !area.inArea(pos)) continue;
                savedRail.area = (V)area;
                area.savedRails.add(savedRail);
                break;
            }
        });
    }
}

