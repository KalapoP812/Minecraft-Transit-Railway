/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntIntImmutablePair
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2LongOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectCollection
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectLongImmutablePair
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet
 */
package org.mtr.core.simulation;

import java.nio.file.Path;
import java.util.UUID;
import java.util.function.Consumer;
import javax.annotation.Nullable;
import org.mtr.core.Main;
import org.mtr.core.data.Client;
import org.mtr.core.data.Data;
import org.mtr.core.data.Depot;
import org.mtr.core.data.Lift;
import org.mtr.core.data.NameColorDataBase;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.Route;
import org.mtr.core.data.Settings;
import org.mtr.core.data.Siding;
import org.mtr.core.data.Station;
import org.mtr.core.data.TransportMode;
import org.mtr.core.data.VehiclePosition;
import org.mtr.core.directions.DirectionsFinder;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.serializer.SerializedDataBaseWithId;
import org.mtr.core.servlet.MessageQueue;
import org.mtr.core.servlet.OperationProcessor;
import org.mtr.core.servlet.QueueObject;
import org.mtr.core.simulation.FileLoader;
import org.mtr.core.tool.Utilities;
import org.mtr.legacy.data.LegacyRailLoader;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntIntImmutablePair;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2LongOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectCollection;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectLongImmutablePair;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet;

public class Simulator
extends Data
implements Utilities {
    private long lastMillis;
    private boolean autoSave = false;
    private long gameMillis;
    private long gameMillisPerDay = 1200000L;
    private boolean isTimeMoving;
    private long lastSetGameMillisMidnight;
    public final ObjectArraySet<Client> clients = new ObjectArraySet();
    public final String dimension;
    public final String[] dimensions;
    public final DirectionsFinder directionsFinder = new DirectionsFinder(this);
    private final FileLoader<Station> fileLoaderStations;
    private final FileLoader<Platform> fileLoaderPlatforms;
    private final FileLoader<Siding> fileLoaderSidings;
    private final FileLoader<Route> fileLoaderRoutes;
    private final FileLoader<Depot> fileLoaderDepots;
    private final FileLoader<Lift> fileLoaderLifts;
    private final FileLoader<Rail> fileLoaderRails;
    private final FileLoader<Settings> fileLoaderSettings;
    private final Consumer<Settings> writeSettings;
    private final MessageQueue<Runnable> queuedRuns = new MessageQueue();
    private final ObjectImmutableList<ObjectArrayList<Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>>>> vehiclePositions;
    private final Object2LongOpenHashMap<UUID> ridingVehicleIds = new Object2LongOpenHashMap();
    private final MessageQueue<QueueObject> messageQueueC2S = new MessageQueue();
    private final MessageQueue<QueueObject> messageQueueS2C = new MessageQueue();
    private static final int SIMULATION_DIFFERENCE_LOGGING_THRESHOLD = 120000;

    public Simulator(String dimension, String[] dimensions, Path rootPath, boolean threadedFileLoading) {
        this.dimension = dimension;
        this.dimensions = dimensions;
        Path savePath = rootPath.resolve(dimension);
        ObjectLongImmutablePair<FileLoaderHolder> fileLoaderHolderAndDuration = Utilities.measureDuration(() -> {
            LegacyRailLoader.load(savePath, (ObjectArraySet<Rail>)this.rails, threadedFileLoading);
            return new FileLoaderHolder(new FileLoader<Station>((ObjectSet<Station>)this.stations, messagePackHelper -> new Station((ReaderBase)messagePackHelper, (Data)this), savePath, "stations", threadedFileLoading), new FileLoader<Platform>((ObjectSet<Platform>)this.platforms, messagePackHelper -> new Platform((ReaderBase)messagePackHelper, (Data)this), savePath, "platforms", threadedFileLoading), new FileLoader<Siding>((ObjectSet<Siding>)this.sidings, messagePackHelper -> new Siding((ReaderBase)messagePackHelper, (Data)this), savePath, "sidings", threadedFileLoading), new FileLoader<Route>((ObjectSet<Route>)this.routes, messagePackHelper -> new Route((ReaderBase)messagePackHelper, (Data)this), savePath, "routes", threadedFileLoading), new FileLoader<Depot>((ObjectSet<Depot>)this.depots, messagePackHelper -> new Depot((ReaderBase)messagePackHelper, (Data)this), savePath, "depots", threadedFileLoading), new FileLoader<Lift>((ObjectSet<Lift>)this.lifts, messagePackHelper -> new Lift((ReaderBase)messagePackHelper, (Data)this), savePath, "lifts", threadedFileLoading), new FileLoader<Rail>((ObjectSet<Rail>)this.rails, Rail::new, savePath, "rails", threadedFileLoading));
        });
        this.fileLoaderStations = ((FileLoaderHolder)fileLoaderHolderAndDuration.left()).fileLoaderStations;
        this.fileLoaderPlatforms = ((FileLoaderHolder)fileLoaderHolderAndDuration.left()).fileLoaderPlatforms;
        this.fileLoaderSidings = ((FileLoaderHolder)fileLoaderHolderAndDuration.left()).fileLoaderSidings;
        this.fileLoaderRoutes = ((FileLoaderHolder)fileLoaderHolderAndDuration.left()).fileLoaderRoutes;
        this.fileLoaderDepots = ((FileLoaderHolder)fileLoaderHolderAndDuration.left()).fileLoaderDepots;
        this.fileLoaderLifts = ((FileLoaderHolder)fileLoaderHolderAndDuration.left()).fileLoaderLifts;
        this.fileLoaderRails = ((FileLoaderHolder)fileLoaderHolderAndDuration.left()).fileLoaderRails;
        Main.LOGGER.info("Data loading complete for {} in {} second(s)", dimension, Float.valueOf((float)fileLoaderHolderAndDuration.rightLong() / 1000.0f));
        this.sync();
        this.depots.forEach(Depot::init);
        this.rails.forEach(Rail::checkMigrationStatus);
        ObjectArrayList tempVehiclePositions = new ObjectArrayList();
        for (int i = 0; i < TransportMode.values().length; ++i) {
            ObjectArrayList vehiclePositionsForTransportMode = new ObjectArrayList();
            vehiclePositionsForTransportMode.add(new Object2ObjectAVLTreeMap());
            vehiclePositionsForTransportMode.add(new Object2ObjectAVLTreeMap());
            tempVehiclePositions.add(vehiclePositionsForTransportMode);
        }
        this.vehiclePositions = new ObjectImmutableList((ObjectList)tempVehiclePositions);
        this.sidings.forEach(siding -> siding.initVehiclePositions((Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>>)((Object2ObjectAVLTreeMap)((ObjectArrayList)this.vehiclePositions.get(siding.getTransportModeOrdinal())).get(1))));
        ObjectArraySet settings = new ObjectArraySet();
        this.fileLoaderSettings = new FileLoader<Settings>((ObjectSet<Settings>)settings, Settings::new, savePath, "settings", threadedFileLoading);
        this.writeSettings = newSettings -> {
            settings.clear();
            settings.add(newSettings);
        };
        this.setCurrentMillis(Utilities.getElement(new ObjectArrayList((ObjectCollection)settings), 0, new Settings(0L)).getLastSimulationMillis());
    }

    public void tick() {
        long totalDifference = System.currentTimeMillis() - this.getCurrentMillis();
        if (totalDifference >= 120000L) {
            if (totalDifference > 3600000L) {
                this.setCurrentMillis(System.currentTimeMillis() - 3600000L);
                this.sidings.forEach(Siding::clearVehicles);
            }
            ObjectLongImmutablePair<Integer> ticksAndDuration = Utilities.measureDuration(this::tickUntilCaughtUp);
            Main.LOGGER.info("Simulation difference of {}h{}m for {} caught up with {} ticks in {} second(s)", (totalDifference / 1000L / 3600L), (totalDifference / 1000L / 60L % 60L), this.dimension, ticksAndDuration.left(), Float.valueOf((float)ticksAndDuration.rightLong() / 1000.0f));
        } else {
            this.tickUntilCaughtUp();
        }
    }

    public void save() {
        this.autoSave = true;
    }

    public void stop() {
        this.save(false);
    }

    public int matchMillis(long millis) {
        if (Utilities.circularDifference(this.getCurrentMillis(), millis, 86400000L) < 0L) {
            return 1;
        }
        return Utilities.circularDifference(millis, this.lastMillis, 86400000L) > 0L ? 0 : -1;
    }

    public void instantDeployDepots(ObjectArrayList<Depot> depotsToInstantDeploy) {
        long oldLastMillis = this.lastMillis;
        long oldCurrentMillis = this.getCurrentMillis();
        for (int i = 0; i < 86400000; i += 1000) {
            this.lastMillis = this.getCurrentMillis();
            this.setCurrentMillis(this.lastMillis + 1000L);
            depotsToInstantDeploy.forEach(depot -> depot.savedRails.forEach(siding -> siding.simulateTrain(1000L, null)));
        }
        this.lastMillis = oldLastMillis;
        this.setCurrentMillis(oldCurrentMillis);
    }

    public void instantDeployDepotsByName(Simulator simulator, String filter) {
        this.instantDeployDepots(NameColorDataBase.getDataByName(simulator.depots, filter));
    }

    public void setGameTime(long gameMillis, long gameMillisPerDay, boolean isTimeMoving) {
        this.gameMillis = gameMillisPerDay > 0L ? gameMillis % gameMillisPerDay : gameMillis;
        this.gameMillisPerDay = gameMillisPerDay;
        this.isTimeMoving = isTimeMoving;
        this.lastSetGameMillisMidnight = this.getCurrentMillis() - gameMillis;
        this.depots.forEach(Depot::generatePlatformDirectionsAndWriteDeparturesToSidings);
    }

    public long getGameMillisPerDay() {
        return this.gameMillisPerDay;
    }

    public boolean isTimeMoving() {
        return this.isTimeMoving;
    }

    public long getMillisOfGameMidnight() {
        return this.gameMillisPerDay > 0L && this.isTimeMoving ? Math.max(0L, this.lastSetGameMillisMidnight - this.lastSetGameMillisMidnight / this.gameMillisPerDay * this.gameMillisPerDay) : 0L;
    }

    public int getHour() {
        return this.gameMillisPerDay > 0L ? (int)(this.gameMillis * 24L / this.gameMillisPerDay) : 0;
    }

    public void run(Runnable runnable) {
        this.queuedRuns.put(runnable);
    }

    public void sendMessageC2S(QueueObject queueObject) {
        this.messageQueueC2S.put(queueObject);
    }

    public <T extends SerializedDataBase> void sendMessageS2C(String key, SerializedDataBase data, @Nullable Consumer<T> consumer, @Nullable Class<T> responseDataClass) {
        this.messageQueueS2C.put(new QueueObject(key, data, consumer == null ? null : responseData -> this.run(() -> consumer.accept(responseData)), responseDataClass));
    }

    public void processMessagesS2C(Consumer<QueueObject> callback) {
        this.messageQueueS2C.process(callback);
    }

    public boolean isRiding(UUID uuid, long vehicleId) {
        return this.ridingVehicleIds.getLong(uuid) == vehicleId;
    }

    public void ride(UUID uuid, long vehicleId) {
        this.ridingVehicleIds.put(uuid, vehicleId);
    }

    public void stopRiding(UUID uuid) {
        this.ridingVehicleIds.removeLong(uuid);
    }

    @Nullable
    public Platform getNextPlatformOfRidingVehicle(UUID uuid) {
        Platform[] platform = new Platform[]{null};
        this.sidings.forEach(siding -> siding.iterateVehiclesAndRidingEntities((vehicleExtraData, vehicleRidingEntity) -> {
            Platform checkPlatform;
            if (vehicleRidingEntity.uuid.equals(uuid) && (checkPlatform = (Platform)this.platformIdMap.get(vehicleExtraData.getNextPlatformId())) != null) {
                platform[0] = checkPlatform;
            }
        }));
        return platform[0];
    }

    private int tickUntilCaughtUp() {
        long totalDifference;
        int ticks = 0;
        while (true) {
            ++ticks;
            totalDifference = System.currentTimeMillis() - this.getCurrentMillis();
            if (totalDifference <= 1000L) break;
            this.tick(1000L);
        }
        this.tick(totalDifference);
        return ticks;
    }

    private void tick(long millisElapsed) {
        this.lastMillis = this.getCurrentMillis();
        this.setCurrentMillis(this.lastMillis + millisElapsed);
        try {
            this.vehiclePositions.forEach(vehiclePositionsForTransportMode -> {
                if (!vehiclePositionsForTransportMode.isEmpty()) {
                    vehiclePositionsForTransportMode.remove(0);
                }
                vehiclePositionsForTransportMode.add(new Object2ObjectAVLTreeMap());
            });
            this.rails.forEach(rail -> rail.tick1(this));
            this.rails.forEach(rail -> rail.tick2(millisElapsed));
            this.depots.forEach(Depot::tick);
            if (this.sidings.removeIf(Siding::tick)) {
                this.sync();
            }
            this.sidings.forEach(siding -> siding.simulateTrain(millisElapsed, (ObjectArrayList<Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>>>)((ObjectArrayList)this.vehiclePositions.get(siding.getTransportModeOrdinal()))));
            this.clients.forEach(client -> client.sendUpdates(this));
            if (this.autoSave) {
                this.save(true);
                this.autoSave = false;
            }
            this.lifts.forEach(lift -> lift.tick(millisElapsed));
            this.queuedRuns.process(Runnable::run);
            this.directionsFinder.tick();
            this.messageQueueC2S.process(queueObject -> queueObject.runCallback(OperationProcessor.process(queueObject.key, queueObject.data, this)));
        }
        catch (Throwable e) {
            Main.LOGGER.fatal("", e);
        }
    }

    private void save(boolean useReducedHash) {
        ObjectLongImmutablePair<Boolean> changedAndDuration = Utilities.measureDuration(() -> {
            boolean changed1 = this.save(this.fileLoaderStations, useReducedHash);
            boolean changed2 = this.save(this.fileLoaderPlatforms, useReducedHash);
            boolean changed3 = this.save(this.fileLoaderSidings, useReducedHash);
            boolean changed4 = this.save(this.fileLoaderRoutes, useReducedHash);
            boolean changed5 = this.save(this.fileLoaderDepots, useReducedHash);
            boolean changed6 = this.save(this.fileLoaderLifts, useReducedHash);
            boolean changed7 = this.save(this.fileLoaderRails, useReducedHash);
            return changed1 || changed2 || changed3 || changed4 || changed5 || changed6 || changed7;
        });
        if (((Boolean)changedAndDuration.left()).booleanValue() || !useReducedHash) {
            Main.LOGGER.info("Save complete for {} in {} second(s)", this.dimension, Float.valueOf((float)changedAndDuration.rightLong() / 1000.0f));
        }
        this.writeSettings.accept(new Settings(this.getCurrentMillis()));
        if (useReducedHash) {
            this.fileLoaderSettings.save(false);
        } else {
            this.save(this.fileLoaderSettings, false);
        }
    }

    private <T extends SerializedDataBaseWithId> boolean save(FileLoader<T> fileLoader, boolean useReducedHash) {
        int deletedCount;
        IntIntImmutablePair saveCounts = fileLoader.save(useReducedHash);
        int changedCount = saveCounts.leftInt();
        if (changedCount > 0) {
            Main.LOGGER.info("- Changed {}: {}", fileLoader.key, changedCount);
        }
        if ((deletedCount = saveCounts.rightInt()) > 0) {
            Main.LOGGER.info("- Deleted {}: {}", fileLoader.key, deletedCount);
        }
        return changedCount > 0 || deletedCount > 0;
    }

    private static class FileLoaderHolder {
        private final FileLoader<Station> fileLoaderStations;
        private final FileLoader<Platform> fileLoaderPlatforms;
        private final FileLoader<Siding> fileLoaderSidings;
        private final FileLoader<Route> fileLoaderRoutes;
        private final FileLoader<Depot> fileLoaderDepots;
        private final FileLoader<Lift> fileLoaderLifts;
        private final FileLoader<Rail> fileLoaderRails;

        private FileLoaderHolder(FileLoader<Station> fileLoaderStations, FileLoader<Platform> fileLoaderPlatforms, FileLoader<Siding> fileLoaderSidings, FileLoader<Route> fileLoaderRoutes, FileLoader<Depot> fileLoaderDepots, FileLoader<Lift> fileLoaderLifts, FileLoader<Rail> fileLoaderRails) {
            this.fileLoaderStations = fileLoaderStations;
            this.fileLoaderPlatforms = fileLoaderPlatforms;
            this.fileLoaderSidings = fileLoaderSidings;
            this.fileLoaderRoutes = fileLoaderRoutes;
            this.fileLoaderDepots = fileLoaderDepots;
            this.fileLoaderLifts = fileLoaderLifts;
            this.fileLoaderRails = fileLoaderRails;
        }
    }
}

