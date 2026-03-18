/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.com.google.gson.JsonElement
 *  org.mtr.libraries.com.google.gson.JsonObject
 *  org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanBooleanImmutablePair
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectOpenHashSet
 */
package org.mtr.core.data;

import java.util.UUID;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Predicate;
import javax.annotation.Nullable;
import org.mtr.core.data.Depot;
import org.mtr.core.data.InterchangeColorsForStationName;
import org.mtr.core.data.InterchangeRouteNamesForColor;
import org.mtr.core.data.NameColorDataBase;
import org.mtr.core.data.PathData;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Route;
import org.mtr.core.data.RouteType;
import org.mtr.core.data.Siding;
import org.mtr.core.data.Station;
import org.mtr.core.data.VehicleCar;
import org.mtr.core.data.VehicleRidingEntity;
import org.mtr.core.generated.data.VehicleExtraDataSchema;
import org.mtr.core.serializer.JsonReader;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.com.google.gson.JsonElement;
import org.mtr.libraries.com.google.gson.JsonObject;
import org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanBooleanImmutablePair;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectOpenHashSet;

public class VehicleExtraData
extends VehicleExtraDataSchema {
    private int stopIndex = -1;
    private double oldStoppingPoint;
    private boolean oldDoorTarget;
    private long oldPowerLevel;
    private double oldSpeedTarget;
    private boolean oldIsCurrentlyManual;
    private boolean hasRidingEntityUpdate;
    public final ObjectImmutableList<PathData> immutablePath;
    public final ObjectImmutableList<VehicleCar> immutableVehicleCars;

    private VehicleExtraData(long depotId, long sidingId, double railLength, double totalVehicleLength, long repeatIndex1, long repeatIndex2, double acceleration, double deceleration, boolean isManualAllowed, double maxManualSpeed, long manualToAutomaticTime, double totalDistance, double defaultPosition, ObjectArrayList<VehicleCar> vehicleCars, ObjectArrayList<PathData> path) {
        super(depotId, sidingId, railLength, totalVehicleLength, repeatIndex1, repeatIndex2, acceleration, deceleration, isManualAllowed, maxManualSpeed, manualToAutomaticTime, totalDistance, defaultPosition);
        this.path.clear();
        this.path.addAll(path);
        this.immutablePath = new ObjectImmutableList(path);
        this.vehicleCars.clear();
        this.vehicleCars.addAll(vehicleCars);
        this.immutableVehicleCars = new ObjectImmutableList(vehicleCars);
    }

    public VehicleExtraData(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
        this.immutablePath = new ObjectImmutableList((ObjectList)this.path);
        this.immutableVehicleCars = new ObjectImmutableList((ObjectList)this.vehicleCars);
    }

    public VehicleExtraData copy(int pathUpdateIndex) {
        VehicleExtraData newVehicleExtraData = new VehicleExtraData(new JsonReader((JsonElement)Utilities.getJsonObjectFromData(this)));
        newVehicleExtraData.path.clear();
        for (int i = pathUpdateIndex; i <= this.path.size(); ++i) {
            if (i == this.path.size() && !this.path.isEmpty()) {
                newVehicleExtraData.path.add(0, this.path.get(0));
                continue;
            }
            PathData pathData = (PathData)this.path.get(i);
            if (i != pathUpdateIndex && !(pathData.getStartDistance() <= this.stoppingPoint)) break;
            newVehicleExtraData.path.add(pathData);
        }
        return newVehicleExtraData;
    }

    public long getDepotId() {
        return this.depotId;
    }

    public long getSidingId() {
        return this.sidingId;
    }

    public long getPreviousRouteId() {
        return this.previousRouteId;
    }

    public long getPreviousPlatformId() {
        return this.previousPlatformId;
    }

    public long getPreviousStationId() {
        return this.previousStationId;
    }

    public int getPreviousRouteColor() {
        return (int)(this.previousRouteColor & 0xFFFFFFL);
    }

    public String getPreviousRouteName() {
        return this.previousRouteName;
    }

    public String getPreviousRouteNumber() {
        return this.previousRouteNumber;
    }

    public RouteType getPreviousRouteType() {
        return this.previousRouteType;
    }

    public Route.CircularState getPreviousRouteCircularState() {
        return this.previousRouteCircularState;
    }

    public String getPreviousStationName() {
        return this.previousStationName;
    }

    public String getPreviousRouteDestination() {
        return this.previousRouteDestination;
    }

    public long getThisRouteId() {
        return this.thisRouteId;
    }

    public long getThisPlatformId() {
        return this.thisPlatformId;
    }

    public long getThisStationId() {
        return this.thisStationId;
    }

    public int getThisRouteColor() {
        return (int)(this.thisRouteColor & 0xFFFFFFL);
    }

    public String getThisRouteName() {
        return this.thisRouteName;
    }

    public String getThisRouteNumber() {
        return this.thisRouteNumber;
    }

    public RouteType getThisRouteType() {
        return this.thisRouteType;
    }

    public Route.CircularState getThisRouteCircularState() {
        return this.thisRouteCircularState;
    }

    public String getThisStationName() {
        return this.thisStationName;
    }

    public String getThisRouteDestination() {
        return this.thisRouteDestination;
    }

    public long getNextRouteId() {
        return this.nextRouteId;
    }

    public long getNextPlatformId() {
        return this.nextPlatformId;
    }

    public long getNextStationId() {
        return this.nextStationId;
    }

    public int getNextRouteColor() {
        return (int)(this.nextRouteColor & 0xFFFFFFL);
    }

    public String getNextRouteName() {
        return this.nextRouteName;
    }

    public String getNextRouteNumber() {
        return this.nextRouteNumber;
    }

    public RouteType getNextRouteType() {
        return this.nextRouteType;
    }

    public Route.CircularState getNextRouteCircularState() {
        return this.nextRouteCircularState;
    }

    public String getNextStationName() {
        return this.nextStationName;
    }

    public String getNextRouteDestination() {
        return this.nextRouteDestination;
    }

    public int getStopIndex() {
        return this.stopIndex;
    }

    public boolean getIsTerminating() {
        return this.isTerminating;
    }

    public double getAcceleration() {
        return this.acceleration;
    }

    public double getDeceleration() {
        return this.deceleration;
    }

    public void iterateInterchanges(BiConsumer<String, InterchangeColorsForStationName> consumer) {
        this.interchangeColorsForStationNameList.forEach(interchangeColorsForStationName -> consumer.accept(interchangeColorsForStationName.getStationName(), (InterchangeColorsForStationName)interchangeColorsForStationName));
    }

    public void iterateRidingEntities(Consumer<VehicleRidingEntity> consumer) {
        this.ridingEntities.forEach(consumer);
    }

    public int getDoorMultiplier() {
        return this.doorTarget ? 1 : -1;
    }

    public double getStoppingPoint() {
        return this.stoppingPoint;
    }

    public int getPowerLevel() {
        return (int)this.powerLevel;
    }

    public double getSpeedTarget() {
        return this.speedTarget;
    }

    public boolean getIsCurrentlyManual() {
        return this.isCurrentlyManual;
    }

    public double getTotalVehicleLength() {
        return this.totalVehicleLength;
    }

    public double getMaxManualSpeed() {
        return this.maxManualSpeed;
    }

    public boolean getIsManualAllowed() {
        return this.isManualAllowed;
    }

    protected double getRailLength() {
        return this.railLength;
    }

    protected int getRepeatIndex1() {
        return (int)this.repeatIndex1;
    }

    protected int getRepeatIndex2() {
        return (int)this.repeatIndex2;
    }

    protected long getManualToAutomaticTime() {
        return this.manualToAutomaticTime;
    }

    protected double getTotalDistance() {
        return this.totalDistance;
    }

    protected double getDefaultPosition() {
        return this.defaultPosition;
    }

    protected void setStoppingPoint(double stoppingPoint) {
        this.stoppingPoint = stoppingPoint;
    }

    protected void setPowerLevel(int powerLevel) {
        this.powerLevel = powerLevel;
    }

    protected void setSpeedTarget(double speedTarget) {
        this.speedTarget = speedTarget;
    }

    protected void setIsCurrentlyManual(boolean isCurrentlyManual) {
        this.isCurrentlyManual = isCurrentlyManual;
    }

    protected void toggleDoors() {
        this.doorTarget = !this.doorTarget;
    }

    protected void openDoors() {
        this.doorTarget = true;
    }

    protected void closeDoors() {
        this.doorTarget = false;
    }

    protected boolean checkForUpdate() {
        boolean needsUpdate = Math.abs(this.stoppingPoint - this.oldStoppingPoint) > 0.01 || this.doorTarget != this.oldDoorTarget || this.powerLevel != this.oldPowerLevel || Math.abs(this.speedTarget - this.oldSpeedTarget) > 0.01 || this.isCurrentlyManual != this.oldIsCurrentlyManual || this.hasRidingEntityUpdate;
        this.oldStoppingPoint = this.stoppingPoint;
        this.oldDoorTarget = this.doorTarget;
        this.oldPowerLevel = this.powerLevel;
        this.oldSpeedTarget = this.speedTarget;
        this.oldIsCurrentlyManual = this.isCurrentlyManual;
        this.hasRidingEntityUpdate = false;
        return needsUpdate;
    }

    protected void setRoutePlatformInfo(@Nullable Depot depot, int currentIndex) {
        if (depot == null) {
            this.previousRouteId = 0L;
            this.previousPlatformId = 0L;
            this.previousStationId = 0L;
            this.previousRouteColor = 0L;
            this.previousRouteName = "";
            this.previousRouteNumber = "";
            this.previousRouteType = RouteType.NORMAL;
            this.previousRouteCircularState = Route.CircularState.NONE;
            this.previousStationName = "";
            this.previousRouteDestination = "";
            this.thisRouteId = 0L;
            this.thisPlatformId = 0L;
            this.thisStationId = 0L;
            this.thisRouteColor = 0L;
            this.thisRouteName = "";
            this.thisRouteNumber = "";
            this.thisRouteType = RouteType.NORMAL;
            this.thisRouteCircularState = Route.CircularState.NONE;
            this.thisStationName = "";
            this.thisRouteDestination = "";
            this.nextRouteId = 0L;
            this.nextPlatformId = 0L;
            this.nextStationId = 0L;
            this.nextRouteColor = 0L;
            this.nextRouteName = "";
            this.nextRouteNumber = "";
            this.nextRouteType = RouteType.NORMAL;
            this.nextRouteCircularState = Route.CircularState.NONE;
            this.nextStationName = "";
            this.nextRouteDestination = "";
        } else {
            Station station;
            int newStopIndex = ((PathData)this.immutablePath.get(currentIndex)).getStopIndex();
            if (newStopIndex == this.stopIndex) {
                return;
            }
            this.stopIndex = newStopIndex;
            VehiclePlatformRouteInfo vehiclePlatformRouteInfo = depot.getVehiclePlatformRouteInfo(newStopIndex);
            this.previousRouteId = VehicleExtraData.getId(vehiclePlatformRouteInfo.previousRoute);
            this.previousPlatformId = VehicleExtraData.getId(vehiclePlatformRouteInfo.previousPlatform);
            this.previousStationId = VehicleExtraData.getStationId(vehiclePlatformRouteInfo.previousPlatform);
            this.previousRouteColor = VehicleExtraData.getColor(vehiclePlatformRouteInfo.previousRoute);
            this.previousRouteName = VehicleExtraData.getName(vehiclePlatformRouteInfo.previousRoute);
            this.previousRouteNumber = VehicleExtraData.getRouteNumber(vehiclePlatformRouteInfo.previousRoute);
            this.previousRouteType = VehicleExtraData.getRouteType(vehiclePlatformRouteInfo.previousRoute);
            this.previousRouteCircularState = VehicleExtraData.getRouteCircularState(vehiclePlatformRouteInfo.previousRoute);
            this.previousStationName = VehicleExtraData.getStationName(vehiclePlatformRouteInfo.previousPlatform);
            this.previousRouteDestination = VehicleExtraData.getRouteDestination(vehiclePlatformRouteInfo.previousRoute, 0);
            this.thisRouteId = VehicleExtraData.getId(vehiclePlatformRouteInfo.thisRoute);
            this.thisPlatformId = VehicleExtraData.getId(vehiclePlatformRouteInfo.thisPlatform);
            this.thisStationId = VehicleExtraData.getStationId(vehiclePlatformRouteInfo.thisPlatform);
            this.thisRouteColor = VehicleExtraData.getColor(vehiclePlatformRouteInfo.thisRoute);
            this.thisRouteName = VehicleExtraData.getName(vehiclePlatformRouteInfo.thisRoute);
            this.thisRouteNumber = VehicleExtraData.getRouteNumber(vehiclePlatformRouteInfo.thisRoute);
            this.thisRouteType = VehicleExtraData.getRouteType(vehiclePlatformRouteInfo.thisRoute);
            this.thisRouteCircularState = VehicleExtraData.getRouteCircularState(vehiclePlatformRouteInfo.thisRoute);
            this.thisStationName = VehicleExtraData.getStationName(vehiclePlatformRouteInfo.thisPlatform);
            this.thisRouteDestination = VehicleExtraData.getRouteDestination(vehiclePlatformRouteInfo.thisRoute, vehiclePlatformRouteInfo.platformIndexInRoute);
            this.nextRouteId = VehicleExtraData.getId(vehiclePlatformRouteInfo.nextRoute);
            this.nextPlatformId = VehicleExtraData.getId(vehiclePlatformRouteInfo.nextPlatform);
            this.nextStationId = VehicleExtraData.getStationId(vehiclePlatformRouteInfo.nextPlatform);
            this.nextRouteColor = VehicleExtraData.getColor(vehiclePlatformRouteInfo.nextRoute);
            this.nextRouteName = VehicleExtraData.getName(vehiclePlatformRouteInfo.nextRoute);
            this.nextRouteNumber = VehicleExtraData.getRouteNumber(vehiclePlatformRouteInfo.nextRoute);
            this.nextRouteType = VehicleExtraData.getRouteType(vehiclePlatformRouteInfo.nextRoute);
            this.nextRouteCircularState = VehicleExtraData.getRouteCircularState(vehiclePlatformRouteInfo.nextRoute);
            this.nextStationName = VehicleExtraData.getStationName(vehiclePlatformRouteInfo.nextPlatform);
            this.nextRouteDestination = VehicleExtraData.getRouteDestination(vehiclePlatformRouteInfo.nextRoute, 0);
            this.isTerminating = vehiclePlatformRouteInfo.thisRoute != null && vehiclePlatformRouteInfo.platformIndexInRoute >= vehiclePlatformRouteInfo.thisRoute.getRoutePlatforms().size() - 1;
            this.interchangeColorsForStationNameList.clear();
            Station station2 = station = vehiclePlatformRouteInfo.nextPlatform == null ? null : (Station)((VehiclePlatformRouteInfo)vehiclePlatformRouteInfo).nextPlatform.area;
            if (station != null) {
                station.getInterchangeStationNameToColorToRouteNamesMap(true).forEach((stationName, colorToRouteNames) -> {
                    InterchangeColorsForStationName interchangeColorsForStationName = new InterchangeColorsForStationName((String)stationName);
                    colorToRouteNames.forEach((color, routeNames) -> {
                        InterchangeRouteNamesForColor interchangeRouteNamesForColor = new InterchangeRouteNamesForColor(color.intValue());
                        interchangeRouteNamesForColor.addRouteNames((ObjectArrayList<String>)routeNames);
                        interchangeColorsForStationName.addColor(interchangeRouteNamesForColor);
                    });
                    this.interchangeColorsForStationNameList.add(interchangeColorsForStationName);
                });
            }
        }
    }

    BooleanBooleanImmutablePair containsDriverAndDoorOverride() {
        boolean containsDriver = false;
        boolean doorOverride = false;
        for (VehicleRidingEntity vehicleRidingEntity : this.ridingEntities) {
            if (vehicleRidingEntity.isDriver()) {
                containsDriver = true;
            }
            if (vehicleRidingEntity.getDoorOverride()) {
                doorOverride = true;
            }
            if (!containsDriver || !doorOverride) continue;
            break;
        }
        return new BooleanBooleanImmutablePair(containsDriver, doorOverride);
    }

    void removeRidingEntitiesIf(Predicate<VehicleRidingEntity> predicate) {
        if (this.ridingEntities.removeIf(predicate)) {
            this.hasRidingEntityUpdate = true;
        }
    }

    void addRidingEntities(ObjectOpenHashSet<VehicleRidingEntity> vehicleRidingEntitiesToAdd) {
        if (this.ridingEntities.addAll(vehicleRidingEntitiesToAdd)) {
            this.hasRidingEntityUpdate = true;
        }
    }

    boolean hasRidingEntity(UUID uuid) {
        return this.ridingEntities.stream().anyMatch(vehicleRidingEntity -> vehicleRidingEntity.uuid.equals(uuid));
    }

    public static VehicleExtraData create(long depotId, long sidingId, double railLength, ObjectArrayList<VehicleCar> vehicleCars, ObjectArrayList<PathData> pathSidingToMainRoute, ObjectArrayList<PathData> pathMainRoute, ObjectArrayList<PathData> pathMainRouteToSiding, PathData defaultPathData, boolean repeatInfinitely, double acceleration, double deceleration, boolean isManualAllowed, double maxManualSpeed, long manualToAutomaticTime) {
        double newRailLength = Siding.getRailLength(railLength);
        double newTotalVehicleLength = Siding.getTotalVehicleLength(vehicleCars);
        ObjectArrayList<PathData> path = VehicleExtraData.createPathData(pathSidingToMainRoute, pathMainRoute, pathMainRouteToSiding, repeatInfinitely, defaultPathData);
        long repeatIndex1 = pathSidingToMainRoute.size();
        long repeatIndex2 = repeatInfinitely ? repeatIndex1 + (long)pathMainRoute.size() : 0L;
        double newAcceleration = Siding.roundAcceleration(acceleration);
        double newDeceleration = Siding.roundAcceleration(deceleration);
        double totalDistance = path.isEmpty() ? 0.0 : (repeatInfinitely && repeatIndex2 < (long)path.size() ? ((PathData)Utilities.getElement(path, (int)repeatIndex2)).getStartDistance() : ((PathData)Utilities.getElement(path, -1)).getEndDistance());
        double defaultPosition = (newRailLength + newTotalVehicleLength) / 2.0;
        return new VehicleExtraData(depotId, sidingId, newRailLength, newTotalVehicleLength, repeatIndex1, repeatIndex2, newAcceleration, newDeceleration, isManualAllowed, Math.max(Utilities.kilometersPerHourToMetersPerMillisecond(1.0), maxManualSpeed), manualToAutomaticTime, totalDistance, defaultPosition, vehicleCars, path);
    }

    private static ObjectArrayList<PathData> createPathData(ObjectArrayList<PathData> pathSidingToMainRoute, ObjectArrayList<PathData> pathMainRoute, ObjectArrayList<PathData> pathMainRouteToSiding, boolean repeatInfinitely, PathData defaultPathData) {
        ObjectArrayList tempPath = new ObjectArrayList();
        if (pathSidingToMainRoute.isEmpty() || pathMainRoute.isEmpty() || !repeatInfinitely && pathMainRouteToSiding.isEmpty()) {
            tempPath.add(defaultPathData);
        } else {
            tempPath.addAll(pathSidingToMainRoute);
            tempPath.addAll(pathMainRoute);
            if (repeatInfinitely) {
                tempPath.add(new PathData(new PathData(new JsonReader((JsonElement)new JsonObject())), ((PathData)Utilities.getElement(pathMainRoute, -1)).getEndDistance(), Double.MAX_VALUE));
            } else {
                tempPath.addAll(pathMainRouteToSiding);
            }
        }
        return tempPath;
    }

    private static long getId(@Nullable NameColorDataBase data) {
        return data == null ? 0L : data.getId();
    }

    private static String getName(@Nullable NameColorDataBase data) {
        return data == null ? "" : data.getName();
    }

    private static int getColor(@Nullable NameColorDataBase data) {
        return data == null ? 0 : data.getColor();
    }

    private static long getStationId(@Nullable Platform platform) {
        return platform == null ? 0L : VehicleExtraData.getId(platform.area);
    }

    private static String getStationName(@Nullable Platform platform) {
        return platform == null ? "" : VehicleExtraData.getName(platform.area);
    }

    private static String getRouteNumber(@Nullable Route route) {
        return route == null ? "" : route.getRouteNumber();
    }

    private static RouteType getRouteType(@Nullable Route route) {
        return route == null ? RouteType.NORMAL : route.getRouteType();
    }

    private static Route.CircularState getRouteCircularState(@Nullable Route route) {
        return route == null ? Route.CircularState.NONE : route.getCircularState();
    }

    private static String getRouteDestination(@Nullable Route route, int stopIndex) {
        return route == null ? "" : route.getDestination(stopIndex);
    }

    public static class VehiclePlatformRouteInfo {
        private final Platform previousPlatform;
        private final Platform thisPlatform;
        private final Platform nextPlatform;
        private final Route previousRoute;
        private final Route thisRoute;
        private final Route nextRoute;
        private final int platformIndexInRoute;

        public VehiclePlatformRouteInfo(@Nullable Platform previousPlatform, @Nullable Platform thisPlatform, @Nullable Platform nextPlatform, @Nullable Route previousRoute, @Nullable Route thisRoute, @Nullable Route nextRoute, int platformIndexInRoute) {
            this.previousPlatform = previousPlatform;
            this.thisPlatform = thisPlatform;
            this.nextPlatform = nextPlatform;
            this.previousRoute = previousRoute;
            this.thisRoute = thisRoute;
            this.nextRoute = nextRoute;
            this.platformIndexInRoute = platformIndexInRoute;
        }
    }
}

