/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanLongImmutablePair
 *  org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2LongAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair
 */
package org.mtr.core.data;

import java.util.Collection;
import java.util.Collections;
import java.util.Random;
import java.util.function.BiConsumer;
import java.util.function.LongConsumer;
import javax.annotation.Nullable;
import org.mtr.core.Main;
import org.mtr.core.data.Data;
import org.mtr.core.data.Depot;
import org.mtr.core.data.PathData;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.Route;
import org.mtr.core.data.RoutePlatformData;
import org.mtr.core.data.Station;
import org.mtr.core.data.TransportMode;
import org.mtr.core.data.Trip;
import org.mtr.core.data.Vehicle;
import org.mtr.core.data.VehicleCar;
import org.mtr.core.data.VehicleExtraData;
import org.mtr.core.data.VehiclePosition;
import org.mtr.core.data.VehicleRidingEntity;
import org.mtr.core.generated.data.SidingSchema;
import org.mtr.core.oba.ArrivalAndDeparture;
import org.mtr.core.oba.Frequency;
import org.mtr.core.oba.OccupancyStatus;
import org.mtr.core.oba.SingleElement;
import org.mtr.core.oba.StopWithArrivalsAndDepartures;
import org.mtr.core.oba.TripDetails;
import org.mtr.core.oba.TripStatus;
import org.mtr.core.operation.ArrivalResponse;
import org.mtr.core.path.SidingPathFinder;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.serializer.WriterBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.ConditionalList;
import org.mtr.core.tool.Utilities;
import org.mtr.legacy.data.DataFixer;
import org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanLongImmutablePair;
import org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2LongAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair;

public final class Siding
extends SidingSchema
implements Utilities {
    private PathData defaultPathData;
    private double timeOffsetForRepeating;
    private final ObjectArrayList<SidingPathFinder<Depot, Siding, Station, Platform>> sidingPathFinderSidingToMainRoute = new ObjectArrayList();
    private final ObjectArrayList<SidingPathFinder<Station, Platform, Depot, Siding>> sidingPathFinderMainRouteToSiding = new ObjectArrayList();
    private final ObjectArrayList<PathData> pathMainRoute = new ObjectArrayList();
    private final ObjectArrayList<PathData> pathSidingToMainRoute = new ObjectArrayList();
    private final ObjectArrayList<PathData> pathMainRouteToSiding = new ObjectArrayList();
    private final ObjectArraySet<Vehicle> vehicles = new ObjectArraySet();
    private final ObjectImmutableList<ReaderBase> vehicleReaders;
    private final ObjectArrayList<Trip> trips = new ObjectArrayList();
    private final Long2ObjectAVLTreeMap<ObjectArraySet<Trip.StopTime>> platformTripStopTimes = new Long2ObjectAVLTreeMap();
    private final LongArrayList departures = new LongArrayList();
    private final LongArrayList tempReturnTimes = new LongArrayList();
    private final ObjectArrayList<TimeSegment> timeSegments = new ObjectArrayList();
    private final Long2LongAVLTreeMap vehicleTimesAlongRoute = new Long2LongAVLTreeMap();
    public static final double ACCELERATION_DEFAULT = 4.0E-6;
    public static final double MAX_ACCELERATION = 2.0E-5;
    public static final double MIN_ACCELERATION = 4.0E-7;
    private static final String KEY_PATH_SIDING_TO_MAIN_ROUTE = "pathSidingToMainRoute";
    private static final String KEY_PATH_MAIN_ROUTE_TO_SIDING = "pathMainRouteToSiding";
    private static final String KEY_VEHICLES = "vehicles";

    public Siding(Position position1, Position position2, double railLength, TransportMode transportMode, Data data) {
        super(Siding.getRailLength(railLength), position1, position2, transportMode, data);
        this.vehicleReaders = ObjectImmutableList.of();
    }

    public Siding(ReaderBase readerBase, Data data) {
        super(DataFixer.convertSiding(readerBase), data);
        readerBase.iterateReaderArray(KEY_PATH_SIDING_TO_MAIN_ROUTE, () -> this.pathSidingToMainRoute.clear(), readerBaseChild -> this.pathSidingToMainRoute.add(new PathData((ReaderBase)readerBaseChild)));
        readerBase.iterateReaderArray(KEY_PATH_MAIN_ROUTE_TO_SIDING, () -> this.pathMainRouteToSiding.clear(), readerBaseChild -> this.pathMainRouteToSiding.add(new PathData((ReaderBase)readerBaseChild)));
        this.vehicleReaders = Siding.savePathDataReaderBase(readerBase, KEY_VEHICLES);
        this.updateData(readerBase);
        DataFixer.unpackSidingVehicleCars(readerBase, this.transportMode, this.railLength, (ObjectArrayList<VehicleCar>)this.vehicleCars);
        DataFixer.unpackSidingMaxVehicles(readerBase, value -> {
            this.maxVehicles = value;
        });
    }

    @Override
    public void updateData(ReaderBase readerBase) {
        super.updateData(readerBase);
        this.vehicles.removeIf(vehicle -> !vehicle.getIsOnRoute());
    }

    @Override
    public void serializeFullData(WriterBase writerBase) {
        super.serializeFullData(writerBase);
        writerBase.writeDataset((Collection<? extends SerializedDataBase>)this.pathSidingToMainRoute, KEY_PATH_SIDING_TO_MAIN_ROUTE);
        writerBase.writeDataset((Collection<? extends SerializedDataBase>)this.pathMainRouteToSiding, KEY_PATH_MAIN_ROUTE_TO_SIDING);
        writerBase.writeDataset((Collection<? extends SerializedDataBase>)this.vehicles, KEY_VEHICLES);
    }

    public void init() {
        this.tick();
        this.generatePathDistancesAndTimeSegments();
        if (this.area != null && this.defaultPathData != null) {
            this.vehicleReaders.forEach(readerBase -> this.vehicles.add(new Vehicle(VehicleExtraData.create(((Depot)this.area).getId(), this.id, this.railLength, (ObjectArrayList<VehicleCar>)this.vehicleCars, this.pathSidingToMainRoute, this.pathMainRoute, this.pathMainRouteToSiding, this.defaultPathData, ((Depot)this.area).getRepeatInfinitely(), this.acceleration, this.deceleration, this.getIsManual(), this.maxManualSpeed, this.manualToAutomaticTime), this, (ReaderBase)readerBase, this.data)));
        }
        this.setAcceleration(this.acceleration);
        this.setDeceleration(this.deceleration);
    }

    public double getRailLength() {
        return this.railLength;
    }

    public ObjectArrayList<VehicleCar> getVehicleCars() {
        return this.vehicleCars;
    }

    public boolean getIsManual() {
        return this.maxVehicles < 0L;
    }

    public boolean getIsUnlimited() {
        return this.maxVehicles == 0L;
    }

    public long getMaxVehicles() {
        return this.getIsManual() ? 1L : this.maxVehicles;
    }

    public int getDelayedVehicleSpeedIncreasePercentage() {
        return (int)this.delayedVehicleSpeedIncreasePercentage;
    }

    public int getDelayedVehicleReduceDwellTimePercentage() {
        return (int)this.delayedVehicleReduceDwellTimePercentage;
    }

    public int getTransportModeOrdinal() {
        return this.transportMode.ordinal();
    }

    public boolean getEarlyVehicleIncreaseDwellTime() {
        return this.earlyVehicleIncreaseDwellTime;
    }

    public double getMaxManualSpeed() {
        return this.maxManualSpeed;
    }

    public int getManualToAutomaticTime() {
        return (int)this.manualToAutomaticTime;
    }

    public double getAcceleration() {
        return this.acceleration;
    }

    public double getDeceleration() {
        return this.deceleration;
    }

    public void setVehicleCars(ObjectArrayList<VehicleCar> newVehicleCars) {
        this.vehicleCars.clear();
        double tempVehicleLength = 0.0;
        for (int i = 0; i < newVehicleCars.size(); ++i) {
            VehicleCar vehicleCar = (VehicleCar)newVehicleCars.get(i);
            if (tempVehicleLength + vehicleCar.getTotalLength(i == 0, true) > this.railLength) break;
            this.vehicleCars.add(vehicleCar);
            tempVehicleLength += vehicleCar.getTotalLength(i == 0, false);
            if (this.vehicleCars.size() >= this.transportMode.maxLength) break;
        }
    }

    public void setIsManual(boolean isManual) {
        this.maxVehicles = this.transportMode.continuousMovement ? 0L : (isManual ? -1L : 1L);
    }

    public void setUnlimitedVehicles(boolean unlimitedVehicles) {
        this.maxVehicles = this.transportMode.continuousMovement ? 0L : (unlimitedVehicles ? 0L : 1L);
    }

    public void setMaxVehicles(int newMaxVehicles) {
        this.maxVehicles = this.transportMode.continuousMovement ? 0L : (long)Math.max(1, newMaxVehicles);
    }

    public void setDelayedVehicleSpeedIncreasePercentage(int delayedVehicleSpeedIncreasePercentage) {
        this.delayedVehicleSpeedIncreasePercentage = Utilities.clamp(delayedVehicleSpeedIncreasePercentage, 0, 100);
    }

    public void setDelayedVehicleReduceDwellTimePercentage(int delayedVehicleReduceDwellTimePercentage) {
        this.delayedVehicleReduceDwellTimePercentage = Utilities.clamp(delayedVehicleReduceDwellTimePercentage, 0, 100);
    }

    public void setEarlyVehicleIncreaseDwellTime(boolean earlyVehicleIncreaseDwellTime) {
        this.earlyVehicleIncreaseDwellTime = earlyVehicleIncreaseDwellTime;
    }

    public void setMaxManualSpeed(double maxManualSpeed) {
        this.maxManualSpeed = maxManualSpeed;
    }

    public void setManualToAutomaticTime(int manualToAutomaticTime) {
        this.manualToAutomaticTime = manualToAutomaticTime;
    }

    public void setAcceleration(double newAcceleration) {
        this.acceleration = this.transportMode.continuousMovement ? 2.0E-5 : Siding.roundAcceleration(newAcceleration);
    }

    public void setDeceleration(double newDeceleration) {
        this.deceleration = this.transportMode.continuousMovement ? 2.0E-5 : Siding.roundAcceleration(newDeceleration);
    }

    public void clearVehicles() {
        this.vehicles.clear();
    }

    public void generateRoute(Platform firstPlatform, @Nullable Platform lastPlatform, int stopIndex, long cruisingAltitude) {
        this.vehicles.clear();
        this.pathSidingToMainRoute.clear();
        this.pathMainRouteToSiding.clear();
        this.sidingPathFinderSidingToMainRoute.clear();
        this.sidingPathFinderSidingToMainRoute.add(new SidingPathFinder(this.data, this, firstPlatform, -1));
        this.sidingPathFinderMainRouteToSiding.clear();
        if (lastPlatform != null) {
            this.sidingPathFinderMainRouteToSiding.add(new SidingPathFinder(this.data, lastPlatform, this, stopIndex));
        }
    }

    public boolean tick() {
        SidingPathFinder.findPathTick(this.pathSidingToMainRoute, this.sidingPathFinderSidingToMainRoute, this.area == null ? 0L : ((Depot)this.area).getCruisingAltitude(), () -> this.finishGeneratingPath(false), (startSavedRail, endSavedRail) -> {
            Main.LOGGER.info("Path not found from {} siding {} to main route", this.getDepotName(), this.name);
            this.finishGeneratingPath(true);
        });
        SidingPathFinder.findPathTick(this.pathMainRouteToSiding, this.sidingPathFinderMainRouteToSiding, this.area == null ? 0L : ((Depot)this.area).getCruisingAltitude(), () -> {
            if (this.area != null && SidingPathFinder.overlappingPaths(((Depot)this.area).getPath(), this.pathMainRouteToSiding)) {
                this.pathMainRouteToSiding.remove(0);
            }
            this.finishGeneratingPath(false);
        }, (startSavedRail, endSavedRail) -> {
            Main.LOGGER.info("Path not found from main route to {} siding {}", this.getDepotName(), this.name);
            this.finishGeneratingPath(true);
        });
        if (this.defaultPathData == null) {
            Rail rail = (Rail)Data.tryGet(this.data.positionsToRail, this.position1, this.position2);
            if (rail != null) {
                this.defaultPathData = new PathData(rail, this.id, 1L, -1L, 0.0, rail.railMath.getLength(), this.position1, rail.getStartAngle(this.position1), this.position2, rail.getStartAngle(this.position2));
            }
            return this.defaultPathData == null;
        }
        return false;
    }

    public void initVehiclePositions(Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>> vehiclePositions) {
        this.vehicles.forEach(vehicle -> vehicle.initVehiclePositions(vehiclePositions));
    }

    public void simulateTrain(long millisElapsed, @Nullable ObjectArrayList<Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>>> vehiclePositions) {
        this.vehicleTimesAlongRoute.clear();
        if (this.area == null) {
            this.vehicles.clear();
            this.pathMainRoute.clear();
            this.pathSidingToMainRoute.clear();
            this.pathMainRouteToSiding.clear();
            return;
        }
        int trainsAtDepot = 0;
        boolean spawnTrain = true;
        ObjectArraySet trainsToRemove = new ObjectArraySet();
        LongArrayList visitedDepartureIndices = new LongArrayList();
        for (Vehicle vehicle : this.vehicles) {
            int departureIndex;
            vehicle.simulate(millisElapsed, vehiclePositions, this.vehicleTimesAlongRoute);
            if (vehicle.closeToDepot()) {
                spawnTrain = false;
            }
            if (vehicle.getIsOnRoute()) {
                if (!this.getIsUnlimited()) {
                    long departureIndex2 = vehicle.getDepartureIndex();
                    if (departureIndex2 < 0L && !this.getIsManual() || departureIndex2 >= (long)this.departures.size() || visitedDepartureIndices.contains(departureIndex2)) {
                        trainsToRemove.add(vehicle);
                    } else {
                        visitedDepartureIndices.add(departureIndex2);
                    }
                }
                if (!this.getIsManual() || !this.departures.isEmpty()) continue;
                this.departures.add(vehicle.getSidingDepartureTime() - 86400000L);
                continue;
            }
            if (++trainsAtDepot > 1) {
                trainsToRemove.add(vehicle);
            } else if (!this.pathSidingToMainRoute.isEmpty() && !this.getIsManual() && (departureIndex = this.matchDeparture()) >= 0 && departureIndex < this.departures.size()) {
                if (!this.transportMode.continuousMovement && this.vehicles.stream().anyMatch(checkVehicle -> checkVehicle.getDepartureIndex() == (long)departureIndex)) {
                    if (millisElapsed <= 3600000L) {
                        Main.LOGGER.debug("Already deployed vehicle from {} for departure index {}", this.getDepotName(), departureIndex);
                    }
                } else {
                    vehicle.startUp(departureIndex, this.departures.getLong(departureIndex));
                }
            }
            if (!this.getIsManual()) continue;
            this.departures.clear();
        }
        if (this.defaultPathData != null && !this.vehicleCars.isEmpty() && spawnTrain && (this.getIsUnlimited() || (long)this.vehicles.size() < this.getMaxVehicles())) {
            this.vehicles.add(new Vehicle(VehicleExtraData.create(((Depot)this.area).getId(), this.id, this.railLength, (ObjectArrayList<VehicleCar>)this.vehicleCars, this.pathSidingToMainRoute, this.pathMainRoute, this.pathMainRouteToSiding, this.defaultPathData, ((Depot)this.area).getRepeatInfinitely(), this.acceleration, this.deceleration, this.getIsManual(), this.maxManualSpeed, this.manualToAutomaticTime), this, this.transportMode, this.data));
        }
        if (!trainsToRemove.isEmpty()) {
            trainsToRemove.forEach(arg_0 -> this.vehicles.remove(arg_0));
        }
    }

    public void startGeneratingDepartures() {
        this.departures.clear();
        this.tempReturnTimes.clear();
        int i = 0;
        while ((long)i < this.maxVehicles) {
            this.tempReturnTimes.add(0L);
            ++i;
        }
    }

    public boolean addDeparture(long departure) {
        if (this.getIsManual()) {
            return false;
        }
        if (this.getIsUnlimited()) {
            this.departures.add(departure);
            return true;
        }
        if (!this.timeSegments.isEmpty() && this.area != null) {
            long journeyTime = this.getJourneyTime();
            for (int i = 0; i < this.tempReturnTimes.size(); ++i) {
                if (departure < this.tempReturnTimes.getLong(i)) continue;
                this.departures.add(departure);
                this.tempReturnTimes.set(i, ((Depot)this.area).getRepeatInfinitely() ? Long.MAX_VALUE : departure + journeyTime);
                return true;
            }
        }
        return false;
    }

    public double getTimeAlongRoute(double railProgress) {
        int index = Utilities.getIndexFromConditionalList(this.timeSegments, railProgress);
        return index < 0 ? -1.0 : ((TimeSegment)this.timeSegments.get(index)).getTimeAlongRoute(railProgress);
    }

    public void updateVehicleRidingEntities(long vehicleId, ObjectArrayList<VehicleRidingEntity> vehicleRidingEntities) {
        for (Vehicle vehicle : this.vehicles) {
            if (vehicle.getId() != vehicleId) continue;
            vehicle.updateRidingEntities(vehicleRidingEntities);
            break;
        }
    }

    public String getDepotName() {
        return this.area == null ? "" : ((Depot)this.area).getName();
    }

    public void iterateVehiclesAndRidingEntities(BiConsumer<VehicleExtraData, VehicleRidingEntity> consumer) {
        this.vehicles.forEach(vehicle -> vehicle.vehicleExtraData.iterateRidingEntities(vehicleRidingEntity -> consumer.accept(vehicle.vehicleExtraData, (VehicleRidingEntity)vehicleRidingEntity)));
    }

    public void getArrivals(long currentMillis, Platform platform, long count, ObjectArrayList<ArrivalResponse> arrivalResponseList) {
        long[] maxArrivalAndCount = new long[]{0L, 0L};
        ObjectArrayList<ArrivalResponse> tempArrivalResponseList = new ObjectArrayList<>();
        this.iterateArrivals(currentMillis, platform.getId(), 0L, 86400000L, (trip, tripStopIndex, stopTime, scheduledArrivalTime, scheduledDepartureTime, predicted, deviation, departureIndex, departureOffset) -> {
            if (scheduledArrivalTime + deviation < maxArrivalAndCount[0] || maxArrivalAndCount[1] < count) {
                ArrivalResponse arrivalResponse = new ArrivalResponse(stopTime.customDestination, scheduledArrivalTime + deviation, scheduledDepartureTime + deviation, deviation, predicted, departureIndex, stopTime.tripStopIndex, trip.route, platform);
                arrivalResponse.setCarDetails(this.getVehicleCars());
                tempArrivalResponseList.add(arrivalResponse);
                maxArrivalAndCount[0] = Math.max(maxArrivalAndCount[0], scheduledArrivalTime + deviation);
                maxArrivalAndCount[1] = maxArrivalAndCount[1] + 1L;
            }
        });
        Collections.sort(tempArrivalResponseList);
        int i = 0;
        while ((long)i < Math.min((long)tempArrivalResponseList.size(), count)) {
            arrivalResponseList.add(tempArrivalResponseList.get(i));
            ++i;
        }
    }

    public void getDeparturesForDirections(long currentMillis, Long2ObjectOpenHashMap<ObjectObjectImmutablePair<Route, LongArrayList>> departures) {
        this.getDeparturesAtEndOfRoute(currentMillis, (route, scheduledDepartureTime, deviation) -> ((LongArrayList)((ObjectObjectImmutablePair)departures.computeIfAbsent(route.getId(), key -> new ObjectObjectImmutablePair(route, new LongArrayList()))).right()).add(scheduledDepartureTime + deviation));
    }

    public void getDeparturesForMap(long currentMillis, Object2ObjectAVLTreeMap<String, Long2ObjectAVLTreeMap<LongArrayList>> departures) {
        this.getDeparturesAtEndOfRoute(currentMillis, (route, scheduledDepartureTime, deviation) -> ((LongArrayList)((Long2ObjectAVLTreeMap)departures.computeIfAbsent(route.getHexId(), key -> new Long2ObjectAVLTreeMap())).computeIfAbsent(deviation, key -> new LongArrayList())).add(scheduledDepartureTime - currentMillis));
    }

    public void getOBAArrivalsAndDeparturesElementsWithTripsUsed(SingleElement<StopWithArrivalsAndDepartures> singleElement, StopWithArrivalsAndDepartures stopWithArrivalsAndDepartures, long currentMillis, Platform platform, int millsBefore, int millisAfter) {
        ObjectAVLTreeSet addedTripIds = new ObjectAVLTreeSet();
        this.iterateArrivals(currentMillis, platform.getId(), millsBefore, millisAfter, (trip, tripStopIndex, stopTime, scheduledArrivalTime, scheduledDepartureTime, predicted, deviation, departureIndex, departureOffset) -> {
            String tripId = trip.getTripId(departureIndex, departureOffset);
            stopWithArrivalsAndDepartures.add(ArrivalAndDeparture.create(trip, tripId, platform, stopTime, this.transportMode.continuousMovement ? currentMillis + 8000L : scheduledArrivalTime, this.transportMode.continuousMovement ? currentMillis + 8000L : scheduledDepartureTime, predicted, deviation, this.getOBAOccupancyStatus(predicted), this.getOBAVehicleId(departureIndex), this.getOBAFrequencyElement(currentMillis), new TripStatus(tripId, stopTime, "", "", this.getOBAOccupancyStatus(predicted), predicted, currentMillis, deviation, this.getOBAVehicleId(departureIndex), this.getOBAFrequencyElement(currentMillis))));
            if (!addedTripIds.contains(tripId)) {
                singleElement.addTrip(trip.getOBATripElement(tripId, departureIndex));
                addedTripIds.add(tripId);
            }
        });
        stopWithArrivalsAndDepartures.sort();
    }

    public void getOBATripDetailsWithDataUsed(SingleElement<TripDetails> singleElement, long currentMillis, int tripIndex, int departureIndex, long departureOffset) {
        Trip trip = (Trip)Utilities.getElement(this.trips, tripIndex);
        if (trip != null) {
            trip.getOBATripDetailsWithDataUsed(singleElement, currentMillis, Utilities.getElement(this.departures, departureIndex, 0L) + departureOffset * this.getRepeatInterval(86400000L), departureIndex, departureOffset, (Trip)Utilities.getElement(this.trips, tripIndex + 1), (Trip)Utilities.getElement(this.trips, tripIndex - 1));
        }
    }

    public TripStatus getOBATripStatus(long currentMillis, Trip.StopTime stopTime, int departureIndex, long departureOffset, String closestStop, String nextStop) {
        BooleanLongImmutablePair predictedAndDeviation = this.getPredictedAndDeviation(currentMillis, departureIndex, departureOffset);
        boolean predicted = predictedAndDeviation.leftBoolean();
        long deviation = predictedAndDeviation.rightLong();
        return new TripStatus(stopTime.trip.getTripId(departureIndex, departureOffset), stopTime, closestStop, nextStop, this.getOBAOccupancyStatus(predicted), predicted, currentMillis, deviation, this.getOBAVehicleId(departureIndex), this.getOBAFrequencyElement(currentMillis));
    }

    @Nullable
    public Frequency getOBAFrequencyElement(long currentMillis) {
        return this.transportMode.continuousMovement ? new Frequency(currentMillis) : null;
    }

    long getJourneyTime() {
        TimeSegment lastTimeSegment = (TimeSegment)Utilities.getElement(this.timeSegments, -1);
        return lastTimeSegment == null ? 0L : (long)Math.ceil(lastTimeSegment.startTime + lastTimeSegment.startSpeed / lastTimeSegment.acceleration);
    }

    void writePathCache() {
        PathData.writePathCache(this.pathSidingToMainRoute, this.data, this.transportMode);
        PathData.writePathCache(this.pathMainRouteToSiding, this.data, this.transportMode);
    }

    long getRepeatInterval(long defaultAmount) {
        if (this.area == null) {
            return defaultAmount;
        }
        if (this.transportMode.continuousMovement) {
            return 8000L * (long)((Depot)this.area).savedRails.size();
        }
        if (((Depot)this.area).getRepeatInfinitely()) {
            return Math.round(this.timeOffsetForRepeating);
        }
        if (this.data instanceof Simulator && !((Depot)this.area).getUseRealTime()) {
            return ((Simulator)this.data).getGameMillisPerDay() * ((Depot)this.area).getRepeatDepartures();
        }
        return defaultAmount;
    }

    private void getDeparturesAtEndOfRoute(long currentMillis, DepartureConsumer departureConsumer) {
        if (this.area != null) {
            for (int i = 0; i < ((Depot)this.area).routes.size(); ++i) {
                int targetTripStopIndex;
                long targetRouteId;
                RoutePlatformData nextRoutePlatformData;
                Route route = (Route)((Depot)this.area).routes.get(i);
                RoutePlatformData routePlatformData = (RoutePlatformData)Utilities.getElement(route.getRoutePlatforms(), -1);
                if (routePlatformData == null) continue;
                Route nextRoute = (Route)Utilities.getElement(((Depot)this.area).routes, ((Depot)this.area).getRepeatInfinitely() && i == ((Depot)this.area).routes.size() - 1 ? 0 : i + 1);
                RoutePlatformData routePlatformData2 = nextRoutePlatformData = nextRoute == null ? null : (RoutePlatformData)Utilities.getElement(nextRoute.getRoutePlatforms(), 0);
                if (nextRoutePlatformData != null && routePlatformData.platform.getId() == nextRoutePlatformData.platform.getId()) {
                    targetRouteId = nextRoute.getId();
                    targetTripStopIndex = 0;
                } else {
                    targetRouteId = route.getId();
                    targetTripStopIndex = route.getRoutePlatforms().size() - 1;
                }
                if (this.transportMode.continuousMovement) continue;
                this.iterateArrivals(currentMillis, routePlatformData.platform.getId(), 0L, 86400000L, (trip, tripStopIndex, stopTime, scheduledArrivalTime, scheduledDepartureTime, predicted, deviation, departureIndex, departureOffset) -> {
                    if (trip.route.getId() == targetRouteId && tripStopIndex == targetTripStopIndex) {
                        departureConsumer.accept(route, scheduledDepartureTime, deviation);
                    }
                });
            }
        }
    }

    private int matchDeparture() {
        long repeatInterval = this.getRepeatInterval(0L);
        long offset = this.departures.isEmpty() || repeatInterval == 0L ? 0L : (this.data.getCurrentMillis() - this.departures.getLong(0)) / repeatInterval * repeatInterval;
        for (int i = 0; i < this.departures.size(); ++i) {
            if ((this.data instanceof Simulator ? ((Simulator)this.data).matchMillis(this.departures.getLong(i) + offset) : 0) != 0) continue;
            return i;
        }
        return -1;
    }

    private void finishGeneratingPath(boolean failed) {
        if (failed && this.area != null) {
            ((Depot)this.area).sidingPathGenerationFailed();
        }
        if (this.sidingPathFinderSidingToMainRoute.isEmpty() && this.sidingPathFinderMainRouteToSiding.isEmpty()) {
            this.writePathCache();
            this.generatePathDistancesAndTimeSegments();
            if (this.area != null) {
                ((Depot)this.area).finishGeneratingPath(this.id);
            }
        }
    }

    private BooleanLongImmutablePair getPredictedAndDeviation(long currentMillis, int departureIndex, long departureOffset) {
        long deviation;
        boolean predicted;
        if (this.transportMode.continuousMovement) {
            predicted = true;
            deviation = 0L;
        } else {
            long timeAlongRoute = this.vehicleTimesAlongRoute.getOrDefault((long)departureIndex, -1L);
            predicted = timeAlongRoute >= 0L;
            long repeatInterval = this.getRepeatInterval(86400000L);
            deviation = predicted ? Utilities.circularDifference(currentMillis - repeatInterval * departureOffset - this.departures.getLong(this.getIsManual() ? 0 : departureIndex), timeAlongRoute, repeatInterval) : 0L;
        }
        return new BooleanLongImmutablePair(predicted, deviation);
    }

    private void iterateArrivals(long currentMillis, long platformId, long millsBefore, long millisAfter, ArrivalConsumer arrivalConsumer) {
        if (this.area == null || this.departures.isEmpty()) {
            return;
        }
        ObjectArraySet<Trip.StopTime> tripStopTimes = this.platformTripStopTimes.get(platformId);
        if (tripStopTimes == null) {
            return;
        }
        long repeatInterval = this.getRepeatInterval(86400000L);
        tripStopTimes.forEach(stopTime -> {
            Trip trip = stopTime.trip;
            if (!((Depot)this.area).getRepeatInfinitely() || trip.tripIndexInBlock < this.trips.size() - 1 || stopTime.tripStopIndex < trip.route.getRoutePlatforms().size() - 1) {
                block0: for (int departureIndex = 0; departureIndex < this.departures.size(); ++departureIndex) {
                    long departure = this.departures.getLong(departureIndex);
                    long departureOffset = (currentMillis - (this.transportMode.continuousMovement ? 0L : millsBefore) - repeatInterval / 2L - stopTime.endTime - departure) / repeatInterval + 1L;
                    BooleanLongImmutablePair predictedAndDeviation = this.getPredictedAndDeviation(currentMillis, this.getIsManual() ? -1 : departureIndex, departureOffset);
                    boolean predicted = predictedAndDeviation.leftBoolean();
                    long deviation = predictedAndDeviation.rightLong();
                    while (true) {
                        long scheduledDepartureTime;
                        long scheduledArrivalTime;
                        if (this.transportMode.continuousMovement) {
                            scheduledArrivalTime = 0L;
                            scheduledDepartureTime = 0L;
                        } else {
                            long offsetMillis = repeatInterval * departureOffset;
                            scheduledArrivalTime = stopTime.startTime + offsetMillis + departure;
                            scheduledDepartureTime = stopTime.endTime + offsetMillis + departure;
                        }
                        ++departureOffset;
                        if (scheduledArrivalTime > currentMillis + millisAfter + repeatInterval / 2L) continue block0;
                        if (!this.transportMode.continuousMovement) {
                            boolean missedDeparture;
                            boolean outOfRange = scheduledDepartureTime + deviation < currentMillis - millsBefore || scheduledArrivalTime + deviation > currentMillis + millisAfter;
                            boolean bl = missedDeparture = !predicted && scheduledArrivalTime - stopTime.startTime + 1000L < currentMillis;
                            if (outOfRange || missedDeparture) {
                                if (!this.getIsManual()) continue;
                                continue block0;
                            }
                        }
                        arrivalConsumer.accept(trip, stopTime.tripStopIndex, (Trip.StopTime)stopTime, scheduledArrivalTime, scheduledDepartureTime, predicted, deviation, this.getIsManual() ? -1 : departureIndex, departureOffset - 1L);
                        if (this.transportMode.continuousMovement || this.getIsManual()) break;
                    }
                }
            }
        });
    }

    private OccupancyStatus getOBAOccupancyStatus(boolean predicted) {
        return predicted ? OccupancyStatus.values()[new Random().nextInt(OccupancyStatus.values().length - 2)] : OccupancyStatus.NO_DATA_AVAILABLE;
    }

    private String getOBAVehicleId(int departureIndex) {
        ObjectArrayList<String> vehicleIds = new ObjectArrayList<>();
        this.vehicleCars.forEach(vehicleCar -> {
            String trimmedVehicleId;
            String vehicleId = vehicleCar.getVehicleId();
            int index = vehicleId.lastIndexOf("_");
            String string = trimmedVehicleId = index < 0 ? vehicleId : vehicleId.substring(0, index);
            if (!vehicleIds.contains(trimmedVehicleId)) {
                vehicleIds.add(trimmedVehicleId);
            }
        });
        return vehicleIds.isEmpty() ? "" : String.format("%s_%s", String.join((CharSequence)"_", (Iterable<? extends CharSequence>)vehicleIds), departureIndex);
    }

    private void generatePathDistancesAndTimeSegments() {
        this.vehicles.clear();
        this.pathMainRoute.clear();
        this.trips.clear();
        this.platformTripStopTimes.clear();
        this.timeSegments.clear();
        if (this.pathSidingToMainRoute.isEmpty() || this.area == null || ((Depot)this.area).getPath().isEmpty() || !((Depot)this.area).getRepeatInfinitely() && this.pathMainRouteToSiding.isEmpty()) {
            this.pathSidingToMainRoute.clear();
            this.pathMainRouteToSiding.clear();
        } else {
            this.pathMainRoute.addAll(((Depot)this.area).getPath());
            boolean overlappingFromRepeating = SidingPathFinder.overlappingPaths(this.pathMainRoute, this.pathMainRoute);
            double totalVehicleLength = Siding.getTotalVehicleLength((ObjectArrayList<VehicleCar>)this.vehicleCars);
            if (SidingPathFinder.overlappingPaths(this.pathSidingToMainRoute, this.pathMainRoute)) {
                PathData pathData = (PathData)this.pathMainRoute.remove(0);
                if (((Depot)this.area).getRepeatInfinitely() && !overlappingFromRepeating) {
                    this.pathMainRoute.add(pathData);
                }
            } else if (((Depot)this.area).getRepeatInfinitely() && overlappingFromRepeating) {
                this.pathSidingToMainRoute.add(this.pathMainRoute.remove(0));
            }
            if (SidingPathFinder.overlappingPaths(this.pathMainRoute, this.pathMainRouteToSiding)) {
                this.pathMainRouteToSiding.remove(0);
            }
            SidingPathFinder.generatePathDataDistances(this.pathSidingToMainRoute, 0.0);
            SidingPathFinder.generatePathDataDistances(this.pathMainRoute, ((PathData)Utilities.getElement(this.pathSidingToMainRoute, -1)).getEndDistance());
            SidingPathFinder.generatePathDataDistances(this.pathMainRouteToSiding, ((PathData)Utilities.getElement(this.pathMainRoute, -1)).getEndDistance());
            ObjectArrayList<PathData> path = new ObjectArrayList<>();
            path.addAll(this.pathSidingToMainRoute);
            path.addAll(this.pathMainRoute);
            path.addAll(this.pathMainRouteToSiding);
            double totalDistance = ((PathData)Utilities.getElement(path, -1)).getEndDistance();
            DoubleArrayList stoppingDistances = new DoubleArrayList();
            for (PathData pathData : path) {
                if (pathData.getDwellTime() <= 0L) continue;
                stoppingDistances.add(pathData.getEndDistance());
            }
            ObjectArrayList routePlatformInfoList = new ObjectArrayList();
            for (int i = 0; i < ((Depot)this.area).routes.size(); ++i) {
                Route route = (Route)((Depot)this.area).routes.get(i);
                route.durations.clear();
                for (int j = 0; j < route.getRoutePlatforms().size(); ++j) {
                    long platformId = ((RoutePlatformData)route.getRoutePlatforms().get((int)j)).platform.getId();
                    if (j == 0 && !routePlatformInfoList.isEmpty() && ((RoutePlatformInfo)Utilities.getElement(routePlatformInfoList, -1)).platformId == platformId) {
                        routePlatformInfoList.remove(routePlatformInfoList.size() - 1);
                    }
                    routePlatformInfoList.add(new RoutePlatformInfo(route, i, platformId, route.getDestination(j)));
                }
            }
            double railProgress = (this.railLength + totalVehicleLength) / 2.0;
            double nextStoppingDistance = 0.0;
            double speed = 0.0;
            double time = 0.0;
            LongConsumer writeRouteDuration = null;
            int tripStopIndex = 0;
            for (int i = 0; i < path.size(); ++i) {
                if (railProgress >= nextStoppingDistance) {
                    nextStoppingDistance = stoppingDistances.isEmpty() ? totalDistance : stoppingDistances.removeDouble(0);
                }
                if (i == this.pathSidingToMainRoute.size()) {
                    this.timeOffsetForRepeating = time;
                }
                PathData pathData = (PathData)path.get(i);
                double currentDistance = pathData.getEndDistance();
                while (railProgress < currentDistance) {
                    int speedChange;
                    if (nextStoppingDistance - railProgress + 1.0 < 0.5 * speed * speed / this.deceleration) {
                        speed = Math.max(speed - this.deceleration, this.deceleration);
                        speedChange = -1;
                    } else {
                        double upcomingSlowerSpeed = Siding.getUpcomingSlowerSpeed((ObjectList<PathData>)path, i, railProgress, speed, this.deceleration);
                        if (upcomingSlowerSpeed >= 0.0 && speed > upcomingSlowerSpeed) {
                            speed = Math.max(speed - this.deceleration, upcomingSlowerSpeed);
                            speedChange = -1;
                        } else {
                            double railSpeed = pathData.getSpeedLimitMetersPerMillisecond();
                            if (railSpeed < speed) {
                                speed = Math.max(speed - this.deceleration, railSpeed);
                                speedChange = -1;
                            } else if (railSpeed > speed) {
                                speed = Math.min(speed + this.acceleration, railSpeed);
                                speedChange = 1;
                            } else {
                                speedChange = 0;
                            }
                        }
                    }
                    if (this.timeSegments.isEmpty() || ((TimeSegment)Utilities.getElement(this.timeSegments, -1)).speedChange != speedChange) {
                        this.timeSegments.add(new TimeSegment(railProgress, speed, time, speedChange, this.acceleration, this.deceleration));
                    }
                    railProgress = Math.min(railProgress + speed, currentDistance);
                    time += 1.0;
                }
                if (pathData.getSavedRailBaseId() != 0L) {
                    RoutePlatformInfo routePlatformInfo;
                    long startTime = Math.round(time);
                    long endTime = Math.round(time += (double)pathData.getDwellTime());
                    while (!routePlatformInfoList.isEmpty() && (routePlatformInfo = (RoutePlatformInfo)routePlatformInfoList.get(0)).platformId == pathData.getSavedRailBaseId()) {
                        Trip currentTrip;
                        if (!this.platformTripStopTimes.containsKey(pathData.getSavedRailBaseId())) {
                            this.platformTripStopTimes.put(pathData.getSavedRailBaseId(), new ObjectArraySet());
                        }
                        if ((currentTrip = (Trip)Utilities.getElement(this.trips, -1)) == null || routePlatformInfo.routeIndex != currentTrip.routeIndex) {
                            Trip trip = new Trip(routePlatformInfo.route, routePlatformInfo.routeIndex, this.trips.size(), this);
                            tripStopIndex = 0;
                            ((ObjectArraySet)this.platformTripStopTimes.get(pathData.getSavedRailBaseId())).add(trip.addStopTime(startTime, endTime, pathData.getSavedRailBaseId(), 0, routePlatformInfo.customDestination));
                            this.trips.add(trip);
                        } else {
                            ((ObjectArraySet)this.platformTripStopTimes.get(pathData.getSavedRailBaseId())).add(currentTrip.addStopTime(startTime, endTime, pathData.getSavedRailBaseId(), tripStopIndex, routePlatformInfo.customDestination));
                        }
                        if (writeRouteDuration != null) {
                            writeRouteDuration.accept(startTime);
                        }
                        RoutePlatformInfo finalRoutePlatformInfo = routePlatformInfo;
                        long finalEndTime = endTime;
                        writeRouteDuration = newStartTime -> finalRoutePlatformInfo.route.durations.add(newStartTime - finalEndTime);
                        ++tripStopIndex;
                        routePlatformInfoList.remove(0);
                    }
                } else {
                    time += (double)pathData.getDwellTime();
                }
                if (i + 1 >= path.size() || !pathData.isOppositeRail((PathData)path.get(i + 1))) continue;
                railProgress += totalVehicleLength;
            }
            this.timeOffsetForRepeating = time - this.timeOffsetForRepeating;
        }
    }

    public static double getRailLength(double rawRailLength) {
        return Utilities.round(rawRailLength, 3);
    }

    public static ObjectImmutableList<ReaderBase> savePathDataReaderBase(ReaderBase readerBase, String key) {
        ObjectArrayList tempReaders = new ObjectArrayList();
        readerBase.iterateReaderArray(key, () -> ((ObjectArrayList)tempReaders).clear(), arg_0 -> ((ObjectArrayList)tempReaders).add(arg_0));
        return new ObjectImmutableList((ObjectList)tempReaders);
    }

    public static double getTotalVehicleLength(ObjectArrayList<VehicleCar> vehicleCars) {
        double totalVehicleLength = 0.0;
        for (int i = 0; i < vehicleCars.size(); ++i) {
            totalVehicleLength += ((VehicleCar)vehicleCars.get(i)).getTotalLength(i == 0, i == vehicleCars.size() - 1);
        }
        return totalVehicleLength;
    }

    public static double roundAcceleration(double acceleration) {
        double tempAcceleration = Utilities.round(acceleration, 8);
        return tempAcceleration <= 0.0 ? 4.0E-6 : Utilities.clamp(tempAcceleration, 4.0E-7, 2.0E-5);
    }

    public static double getUpcomingSlowerSpeed(ObjectList<PathData> path, int currentIndex, double railProgress, double currentSpeed, double deceleration) {
        double stoppingDistance = 0.5 * currentSpeed * currentSpeed / deceleration;
        int index = currentIndex + 1;
        double railSpeed = -1.0;
        double bestDistance = 0.0;
        PathData pathData;
        while ((pathData = (PathData)Utilities.getElement(path, index)) != null) {
            double newRailSpeed = pathData.getSpeedLimitMetersPerMillisecond();
            double distance = pathData.getStartDistance() - railProgress;
            if (newRailSpeed < currentSpeed && distance >= bestDistance && distance <= 0.5 * (currentSpeed * currentSpeed - newRailSpeed * newRailSpeed) / deceleration) {
                railSpeed = newRailSpeed;
                bestDistance = distance;
            }
            if (pathData.getEndDistance() >= railProgress + stoppingDistance) {
                return railSpeed;
            }
            ++index;
        }
        return -1.0;
    }

    @FunctionalInterface
    private static interface DepartureConsumer {
        public void accept(Route var1, long var2, long var4);
    }

    @FunctionalInterface
    private static interface ArrivalConsumer {
        public void accept(Trip var1, int var2, Trip.StopTime var3, long var4, long var6, boolean var8, long var9, int var11, long var12);
    }

    private static class TimeSegment
    implements ConditionalList {
        private final double startRailProgress;
        private final double startSpeed;
        private final double startTime;
        private final int speedChange;
        private final double acceleration;
        private final double deceleration;

        private TimeSegment(double startRailProgress, double startSpeed, double startTime, int speedChange, double acceleration, double deceleration) {
            this.startRailProgress = startRailProgress;
            this.startSpeed = startSpeed;
            this.startTime = startTime;
            this.speedChange = speedChange;
            this.acceleration = Siding.roundAcceleration(acceleration);
            this.deceleration = Siding.roundAcceleration(deceleration);
        }

        @Override
        public boolean matchesCondition(double value) {
            return value >= this.startRailProgress;
        }

        private double getTimeAlongRoute(double railProgress) {
            double distance = railProgress - this.startRailProgress;
            if (this.speedChange == 0) {
                return this.startTime + distance / this.startSpeed;
            }
            double totalAcceleration = (double)this.speedChange * (this.speedChange > 0 ? this.acceleration : this.deceleration);
            double endSpeedSquared = 2.0 * totalAcceleration * distance + this.startSpeed * this.startSpeed;
            return endSpeedSquared < 0.0 ? -1.0 : this.startTime + (distance == 0.0 ? 0.0 : (Math.sqrt(endSpeedSquared) - this.startSpeed) / totalAcceleration);
        }
    }

    private static class RoutePlatformInfo {
        private final Route route;
        private final int routeIndex;
        private final long platformId;
        private final String customDestination;

        private RoutePlatformInfo(Route route, int routeIndex, long platformId, @Nullable String customDestination) {
            this.route = route;
            this.routeIndex = routeIndex;
            this.platformId = platformId;
            this.customDestination = customDestination == null ? "" : customDestination;
        }
    }
}

