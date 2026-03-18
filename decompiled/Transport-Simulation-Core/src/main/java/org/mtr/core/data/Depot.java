/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongListIterator
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectCollection
 */
package org.mtr.core.data;

import java.util.Collection;
import java.util.Collections;
import java.util.function.IntConsumer;
import javax.annotation.Nullable;
import org.mtr.core.Main;
import org.mtr.core.data.Data;
import org.mtr.core.data.PathData;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Route;
import org.mtr.core.data.RoutePlatformData;
import org.mtr.core.data.Siding;
import org.mtr.core.data.Station;
import org.mtr.core.data.TransportMode;
import org.mtr.core.data.VehicleExtraData;
import org.mtr.core.generated.data.DepotSchema;
import org.mtr.core.operation.UpdateDataResponse;
import org.mtr.core.path.SidingPathFinder;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.serializer.WriterBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.Angle;
import org.mtr.core.tool.Utilities;
import org.mtr.legacy.data.DataFixer;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongListIterator;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectCollection;

public final class Depot
extends DepotSchema
implements Utilities {
    @Nullable
    private OnGenerationComplete onGenerationComplete;
    private long repeatDepartures;
    public final ObjectArrayList<Route> routes = new ObjectArrayList();
    private final ObjectArrayList<PathData> path = new ObjectArrayList();
    private final ObjectArrayList<PlatformRouteDetails> platformsInRoute = new ObjectArrayList();
    private final ObjectArrayList<SidingPathFinder<Station, Platform, Station, Platform>> sidingPathFinders = new ObjectArrayList();
    private final LongAVLTreeSet generatingSidingIds = new LongAVLTreeSet();
    public static final int CONTINUOUS_MOVEMENT_FREQUENCY = 8000;
    private static final String KEY_PATH = "path";

    public Depot(TransportMode transportMode, Data data) {
        super(transportMode, data);
    }

    public Depot(ReaderBase readerBase, Data data) {
        super(readerBase, data);
        readerBase.iterateReaderArray(KEY_PATH, () -> this.path.clear(), readerBaseChild -> this.path.add(new PathData((ReaderBase)readerBaseChild)));
        super.updateData(readerBase);
        DataFixer.unpackDepotDepartures(readerBase, this.realTimeDepartures);
    }

    @Override
    public void updateData(ReaderBase readerBase) {
        long tempLastGeneratedMillis = this.lastGeneratedMillis;
        GeneratedStatus tempLastGeneratedStatus = this.lastGeneratedStatus;
        long tempLastGeneratedFailedStartId = this.lastGeneratedFailedStartId;
        long tempLastGeneratedFailedEndId = this.lastGeneratedFailedEndId;
        long tempLastGeneratedFailedSidingCount = this.lastGeneratedFailedSidingCount;
        super.updateData(readerBase);
        if (this.data instanceof Simulator) {
            this.lastGeneratedMillis = tempLastGeneratedMillis;
            this.lastGeneratedStatus = tempLastGeneratedStatus;
            this.lastGeneratedFailedStartId = tempLastGeneratedFailedStartId;
            this.lastGeneratedFailedEndId = tempLastGeneratedFailedEndId;
            this.lastGeneratedFailedSidingCount = tempLastGeneratedFailedSidingCount;
        }
    }

    @Override
    public void serializeFullData(WriterBase writerBase) {
        super.serializeFullData(writerBase);
        writerBase.writeDataset((Collection<? extends SerializedDataBase>)this.path, KEY_PATH);
    }

    public void init() {
        this.writePathCache();
        this.savedRails.forEach(Siding::init);
        this.generatePlatformDirectionsAndWriteDeparturesToSidings();
    }

    public void writePathCache() {
        PathData.writePathCache(this.path, this.data, this.transportMode);
        this.savedRails.forEach(Siding::writePathCache);
    }

    public void setUseRealTime(boolean useRealTime) {
        this.useRealTime = useRealTime;
    }

    public void setFrequency(int hour, int frequency) {
        if (hour >= 0 && hour < 24) {
            while (this.frequencies.size() < 24) {
                this.frequencies.add(0L);
            }
            this.frequencies.set(hour, (long)Math.max(0, frequency));
        }
    }

    public void setRepeatInfinitely(boolean repeatInfinitely) {
        this.repeatInfinitely = repeatInfinitely;
    }

    public void setCruisingAltitude(long cruisingAltitude) {
        this.cruisingAltitude = cruisingAltitude;
    }

    public LongArrayList getRouteIds() {
        return this.routeIds;
    }

    public long getLastGeneratedMillis() {
        return this.lastGeneratedMillis;
    }

    public GeneratedStatus getLastGeneratedStatus() {
        return this.lastGeneratedStatus;
    }

    public void getFailedPlatformIds(GenerationStatusConsumer generationStatusConsumer, IntConsumer lastGeneratedFailedSidingCountConsumer) {
        if (this.lastGeneratedFailedStartId != 0L && this.lastGeneratedFailedEndId != 0L) {
            generationStatusConsumer.accept(this.lastGeneratedFailedStartId, this.lastGeneratedFailedEndId);
        }
        if (this.lastGeneratedFailedSidingCount > 0L) {
            lastGeneratedFailedSidingCountConsumer.accept((int)this.lastGeneratedFailedSidingCount);
        }
    }

    public boolean getRepeatInfinitely() {
        return this.repeatInfinitely;
    }

    public long getCruisingAltitude() {
        return this.cruisingAltitude;
    }

    public boolean getUseRealTime() {
        return this.useRealTime;
    }

    public long getFrequency(int hour) {
        return hour >= 0 && hour < Math.min(24, this.frequencies.size()) ? this.frequencies.getLong(hour) : 0L;
    }

    public LongArrayList getRealTimeDepartures() {
        return this.realTimeDepartures;
    }

    public ObjectArrayList<PathData> getPath() {
        return this.path;
    }

    public void writeRouteCache(Long2ObjectOpenHashMap<Route> routeIdMap) {
        this.routes.clear();
        this.routeIds.forEach(id -> this.routes.add(routeIdMap.get(id)));
        for (int i = this.routes.size() - 1; i >= 0; --i) {
            if (this.routes.get(i) == null) {
                this.routeIds.removeLong(i);
                this.routes.remove(i);
                continue;
            }
            ((Route)this.routes.get((int)i)).depots.add(this);
        }
        this.platformsInRoute.clear();
        long previousPlatformId = 0L;
        for (Route route : this.routes) {
            for (int i = 0; i < route.getRoutePlatforms().size(); ++i) {
                Platform platform = ((RoutePlatformData)route.getRoutePlatforms().get((int)i)).platform;
                if (platform == null || platform.getId() == previousPlatformId) continue;
                this.platformsInRoute.add(new PlatformRouteDetails(platform, i == 0 ? null : route, i == 0 ? Integer.MAX_VALUE : i - 1));
                previousPlatformId = platform.getId();
            }
        }
    }

    public void tick() {
        SidingPathFinder.findPathTick(this.path, this.sidingPathFinders, this.cruisingAltitude, () -> {
            if (!this.platformsInRoute.isEmpty()) {
                this.lastGeneratedFailedSidingCount = 0L;
                PathData.writePathCache(this.path, this.data, this.transportMode);
                this.savedRails.forEach(siding -> {
                    siding.generateRoute(((PlatformRouteDetails)Utilities.getElement(this.platformsInRoute, 0)).platform, this.repeatInfinitely ? null : ((PlatformRouteDetails)Utilities.getElement(this.platformsInRoute, -1)).platform, this.platformsInRoute.size(), this.cruisingAltitude);
                    this.generatingSidingIds.add(siding.getId());
                });
            }
        }, (startSavedRail, endSavedRail) -> this.updateGenerationStatus(GeneratedStatus.PATH_NOT_FOUND, startSavedRail.getId(), endSavedRail.getId(), "Path not found for %s"));
    }

    public void finishGeneratingPath(long sidingId) {
        this.generatingSidingIds.remove(sidingId);
        if (this.generatingSidingIds.isEmpty()) {
            this.updateGenerationStatus(GeneratedStatus.SUCCESSFUL, 0L, 0L, "Path generation complete for %s");
            this.generatePlatformDirectionsAndWriteDeparturesToSidings();
        }
    }

    public VehicleExtraData.VehiclePlatformRouteInfo getVehiclePlatformRouteInfo(int stopIndex) {
        PlatformRouteDetails nextNextData;
        PlatformRouteDetails nextData;
        PlatformRouteDetails thisData;
        PlatformRouteDetails previousData;
        int platformCount = this.platformsInRoute.size();
        if (platformCount == 0) {
            previousData = null;
            thisData = null;
            nextData = null;
            nextNextData = null;
        } else if (this.repeatInfinitely) {
            previousData = (PlatformRouteDetails)Utilities.getElement(this.platformsInRoute, (stopIndex - 1 + platformCount) % platformCount);
            thisData = (PlatformRouteDetails)Utilities.getElement(this.platformsInRoute, stopIndex % platformCount);
            nextData = (PlatformRouteDetails)Utilities.getElement(this.platformsInRoute, (stopIndex + 1) % platformCount);
            nextNextData = (PlatformRouteDetails)Utilities.getElement(this.platformsInRoute, (stopIndex + 2) % platformCount);
        } else {
            previousData = (PlatformRouteDetails)Utilities.getElement(this.platformsInRoute, stopIndex - 1);
            thisData = (PlatformRouteDetails)Utilities.getElement(this.platformsInRoute, stopIndex);
            nextData = (PlatformRouteDetails)Utilities.getElement(this.platformsInRoute, stopIndex + 1);
            nextNextData = (PlatformRouteDetails)Utilities.getElement(this.platformsInRoute, stopIndex + 2);
        }
        return new VehicleExtraData.VehiclePlatformRouteInfo(previousData == null ? null : previousData.platform, thisData == null ? null : thisData.platform, nextData == null ? null : nextData.platform, thisData == null ? null : thisData.route, nextData == null ? null : nextData.route, nextNextData == null ? null : nextNextData.route, nextData == null ? Integer.MAX_VALUE : nextData.platformIndex);
    }

    public void generatePlatformDirectionsAndWriteDeparturesToSidings() {
        Long2ObjectOpenHashMap platformDirections = new Long2ObjectOpenHashMap();
        for (int i = 1; i < this.path.size(); ++i) {
            long platformId2 = ((PathData)this.path.get(i - 1)).getSavedRailBaseId();
            if (platformId2 == 0L) continue;
            Angle newAngle = ((PathData)this.path.get(i)).getFacingStart();
            if (!platformDirections.containsKey(platformId2)) {
                platformDirections.put(platformId2, newAngle);
                continue;
            }
            if (newAngle == platformDirections.get(platformId2)) continue;
            platformDirections.put(platformId2, null);
        }
        platformDirections.forEach((platformId, angle) -> {
            Platform platform = (Platform)this.data.platformIdMap.get(platformId);
            if (platform != null) {
                platform.setAngles(this.id, (Angle)(angle));
            }
        });
        LongArrayList departures = new LongArrayList();
        long gameMillisPerDay = this.data instanceof Simulator ? ((Simulator)this.data).getGameMillisPerDay() : 0L;
        this.repeatDepartures = 1L;
        if (this.transportMode.continuousMovement) {
            for (int i = 0; i < this.savedRails.size(); ++i) {
                departures.add((long)i * 8000L);
            }
        } else if (this.useRealTime) {
            departures.addAll((LongList)this.realTimeDepartures);
        } else if (this.data instanceof Simulator) {
            Simulator simulator = (Simulator)this.data;
            long offsetMillis = simulator.getMillisOfGameMidnight();
            long lastDeparture = Long.MIN_VALUE;
            if (gameMillisPerDay > 0L) {
                long highestJourneyTime = this.savedRails.stream().mapToLong(Siding::getJourneyTime).reduce(0L, Math::max);
                this.repeatDepartures = highestJourneyTime == 0L ? 1L : (long)Math.ceil((double)highestJourneyTime / (double)gameMillisPerDay);
            }
            for (int i = 0; i < 24; ++i) {
                long newDeparture;
                long frequency = this.getFrequency(((Simulator)this.data).isTimeMoving() ? i : ((Simulator)this.data).getHour());
                if (frequency == 0L) continue;
                long intervalMillis = 14400000L / frequency;
                long hourMinMillis = 3600000 * i;
                long hourMaxMillis = 3600000 * (i + 1);
                while ((newDeparture = Math.max(hourMinMillis, lastDeparture + intervalMillis)) < hourMaxMillis) {
                    departures.add(offsetMillis + newDeparture * gameMillisPerDay / 86400000L);
                    lastDeparture = newDeparture;
                }
            }
        }
        ObjectArrayList<Siding> sidingsInDepot = new ObjectArrayList<>((ObjectCollection<Siding>)this.savedRails);
        if (!sidingsInDepot.isEmpty()) {
            Collections.shuffle(sidingsInDepot);
            Collections.sort(sidingsInDepot);
            Collections.sort(departures);
            sidingsInDepot.forEach(Siding::startGeneratingDepartures);
            int sidingIndex = 0;
            int i = 0;
            while ((long)i < this.repeatDepartures) {
                LongListIterator longListIterator = departures.iterator();
                block5: while (longListIterator.hasNext()) {
                    long departure = (Long)longListIterator.next();
                    for (int j = 0; j < sidingsInDepot.size(); ++j) {
                        if (!((Siding)sidingsInDepot.get((sidingIndex + j) % sidingsInDepot.size())).addDeparture(departure + gameMillisPerDay * (long)i)) continue;
                        ++sidingIndex;
                        continue block5;
                    }
                }
                ++i;
            }
        }
    }

    public void updateGenerationStatus(long lastGeneratedMillis, GeneratedStatus lastGeneratedStatus, long lastGeneratedFailedStartId, long lastGeneratedFailedEndId) {
        this.lastGeneratedMillis = lastGeneratedMillis;
        this.lastGeneratedStatus = lastGeneratedStatus;
        this.lastGeneratedFailedStartId = lastGeneratedFailedStartId;
        this.lastGeneratedFailedEndId = lastGeneratedFailedEndId;
    }

    long getRepeatDepartures() {
        return this.repeatDepartures;
    }

    void sidingPathGenerationFailed() {
        ++this.lastGeneratedFailedSidingCount;
    }

    private void generateMainRoute(OnGenerationComplete newOnGenerationComplete) {
        if (this.onGenerationComplete != null) {
            this.onGenerationComplete.accept(true);
        }
        this.onGenerationComplete = newOnGenerationComplete;
        if (this.savedRails.isEmpty()) {
            this.updateGenerationStatus(GeneratedStatus.NO_SIDINGS, 0L, 0L, "No sidings in %s");
        } else {
            Main.LOGGER.info("Starting path generation for {}...", this.name);
            this.path.clear();
            this.sidingPathFinders.clear();
            this.generatingSidingIds.clear();
            for (int i = 0; i < this.platformsInRoute.size() - 1; ++i) {
                this.sidingPathFinders.add(new SidingPathFinder(this.data, ((PlatformRouteDetails)this.platformsInRoute.get(i)).platform, ((PlatformRouteDetails)this.platformsInRoute.get(i + 1)).platform, i));
            }
            if (this.sidingPathFinders.isEmpty()) {
                this.updateGenerationStatus(GeneratedStatus.TWO_PLATFORMS_REQUIRED, 0L, 0L, "At least two platforms are required for path generation");
            }
        }
    }

    private void updateGenerationStatus(GeneratedStatus lastGeneratedStatus, long lastGeneratedFailedStartId, long lastGeneratedFailedEndId, String message) {
        this.updateGenerationStatus(this.data.getCurrentMillis(), lastGeneratedStatus, lastGeneratedFailedStartId, lastGeneratedFailedEndId);
        if (this.onGenerationComplete != null) {
            this.onGenerationComplete.accept(false);
        }
        this.onGenerationComplete = null;
        Main.LOGGER.info("{}", String.format(message, this.name));
    }

    public static void generateDepotsByName(Simulator simulator, String filter) {
        Depot.generateDepots(simulator, Depot.getDataByName(simulator.depots, filter));
    }

    public static void generateDepots(Simulator simulator, ObjectArrayList<Depot> depotsToGenerate) {
        LongAVLTreeSet idsToGenerate = new LongAVLTreeSet();
        UpdateDataResponse updateDataResponse = new UpdateDataResponse(simulator);
        depotsToGenerate.forEach(depot -> {
            idsToGenerate.add(depot.getId());
            depot.generateMainRoute(forceComplete -> {
                idsToGenerate.remove(depot.getId());
                updateDataResponse.addDepot((Depot)depot);
                if (forceComplete || idsToGenerate.isEmpty()) {
                    simulator.sendMessageS2C("generation_status_update", updateDataResponse, null, null);
                }
            });
        });
    }

    public static void clearDepotsByName(Simulator simulator, String filter) {
        Depot.clearDepots(Depot.getDataByName(simulator.depots, filter));
    }

    public static void clearDepots(ObjectArrayList<Depot> depotsToClear) {
        depotsToClear.forEach(depot -> depot.savedRails.forEach(Siding::clearVehicles));
    }

    public static enum GeneratedStatus {
        NONE,
        SUCCESSFUL,
        NO_SIDINGS,
        TWO_PLATFORMS_REQUIRED,
        PATH_NOT_FOUND;

    }

    @FunctionalInterface
    private static interface OnGenerationComplete {
        public void accept(boolean var1);
    }

    @FunctionalInterface
    public static interface GenerationStatusConsumer {
        public void accept(long var1, long var3);
    }

    private static class PlatformRouteDetails {
        private final Platform platform;
        @Nullable
        private final Route route;
        private final int platformIndex;

        private PlatformRouteDetails(Platform platform, @Nullable Route route, int platformIndex) {
            this.platform = platform;
            this.route = route;
            this.platformIndex = platformIndex;
        }
    }
}

