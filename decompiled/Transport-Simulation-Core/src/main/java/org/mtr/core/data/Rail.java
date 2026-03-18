/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2LongAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectOpenHashSet
 */
package org.mtr.core.data;

import java.util.Map;
import org.mtr.core.data.Data;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Position;
import org.mtr.core.data.RailMath;
import org.mtr.core.data.Siding;
import org.mtr.core.data.SignalModification;
import org.mtr.core.data.TransportMode;
import org.mtr.core.generated.data.RailSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.Angle;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2LongAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectOpenHashSet;

public final class Rail
extends RailSchema {
    private long manualBlockCooldown;
    public final double speedLimit1MetersPerMillisecond;
    public final double speedLimit2MetersPerMillisecond;
    public final RailMath railMath;
    private final ObjectOpenHashSet<Rail> connectedRails1 = new ObjectOpenHashSet();
    private final ObjectOpenHashSet<Rail> connectedRails2 = new ObjectOpenHashSet();
    private final Long2LongAVLTreeMap preBlockedVehicleIds = new Long2LongAVLTreeMap();
    private final Long2LongAVLTreeMap currentlyBlockedVehicleIds = new Long2LongAVLTreeMap();
    private final Long2LongAVLTreeMap preBlockedVehicleIdsOld = new Long2LongAVLTreeMap();
    private final Long2LongAVLTreeMap currentlyBlockedVehicleIdsOld = new Long2LongAVLTreeMap();
    private final LongArrayList manualBlockColors = new LongArrayList();
    private final boolean reversePositions;
    private static final int MANUAL_BLOCK_DURATION = 1000;

    public static Rail newRail(Position position1, Angle angle1, Position position2, Angle angle2, Shape shape, double verticalRadius, ObjectArrayList<String> styles, long speedLimit1, long speedLimit2, boolean isPlatform, boolean isSiding, boolean canAccelerate, boolean canConnectRemotely, boolean canHaveSignal, TransportMode transportMode) {
        return new Rail(position1, angle1, position2, angle2, shape, verticalRadius, styles, speedLimit1, speedLimit2, isPlatform, isSiding, canAccelerate, false, canConnectRemotely, canHaveSignal, transportMode);
    }

    public static Rail newTurnBackRail(Position position1, Angle angle1, Position position2, Angle angle2, Shape shape, double verticalRadius, ObjectArrayList<String> styles, TransportMode transportMode) {
        return new Rail(position1, angle1, position2, angle2, shape, verticalRadius, styles, 80L, 80L, false, false, false, true, false, false, transportMode);
    }

    public static Rail newPlatformRail(Position position1, Angle angle1, Position position2, Angle angle2, Shape shape, double verticalRadius, ObjectArrayList<String> styles, TransportMode transportMode) {
        return Rail.newPlatformOrSidingRail(position1, angle1, position2, angle2, shape, verticalRadius, styles, true, transportMode);
    }

    public static Rail newSidingRail(Position position1, Angle angle1, Position position2, Angle angle2, Shape shape, double verticalRadius, ObjectArrayList<String> styles, TransportMode transportMode) {
        return Rail.newPlatformOrSidingRail(position1, angle1, position2, angle2, shape, verticalRadius, styles, false, transportMode);
    }

    private static Rail newPlatformOrSidingRail(Position position1, Angle angle1, Position position2, Angle angle2, Shape shape, double verticalRadius, ObjectArrayList<String> styles, boolean isPlatform, TransportMode transportMode) {
        long speedLimit = isPlatform ? 80L : 40L;
        return new Rail(position1, angle1, position2, angle2, shape, verticalRadius, styles, speedLimit, speedLimit, isPlatform, !isPlatform, false, false, false, true, transportMode);
    }

    public static Rail copy(Rail rail, Shape newShape, double newVerticalRadius) {
        return new Rail(rail.position1, rail.angle1, rail.position2, rail.angle2, newShape, newVerticalRadius, (ObjectArrayList<String>)rail.styles, rail.speedLimit1, rail.speedLimit2, rail.isPlatform, rail.isSiding, rail.canAccelerate, rail.canTurnBack, rail.canConnectRemotely, rail.canHaveSignal, rail.transportMode);
    }

    public static Rail copy(Rail rail, ObjectArrayList<String> newStyles) {
        return new Rail(rail.position1, rail.angle1, rail.position2, rail.angle2, rail.shape, rail.verticalRadius, newStyles, rail.speedLimit1, rail.speedLimit2, rail.isPlatform, rail.isSiding, rail.canAccelerate, rail.canTurnBack, rail.canConnectRemotely, rail.canHaveSignal, rail.transportMode);
    }

    private Rail(Position position1, Angle angle1, Position position2, Angle angle2, Shape shape, double verticalRadius, ObjectArrayList<String> styles, long speedLimit1, long speedLimit2, boolean isPlatform, boolean isSiding, boolean canAccelerate, boolean canTurnBack, boolean canConnectRemotely, boolean canHaveSignal, TransportMode transportMode) {
        super(position1, angle1, position2, angle2, shape, verticalRadius, speedLimit1, speedLimit2, isPlatform, isSiding, canAccelerate, canTurnBack, canConnectRemotely, canHaveSignal, transportMode);
        this.reversePositions = position1.compareTo(position2) > 0;
        this.railMath = this.reversePositions ? new RailMath(position2, angle2, position1, angle1, shape, verticalRadius) : new RailMath(position1, angle1, position2, angle2, shape, verticalRadius);
        this.speedLimit1MetersPerMillisecond = Utilities.kilometersPerHourToMetersPerMillisecond(speedLimit1);
        this.speedLimit2MetersPerMillisecond = Utilities.kilometersPerHourToMetersPerMillisecond(speedLimit2);
        this.styles.addAll(styles);
        this.stylesMigratedLegacy = true;
    }

    public Rail(ReaderBase readerBase) {
        super(readerBase);
        this.reversePositions = this.position1.compareTo(this.position2) > 0;
        this.railMath = this.reversePositions ? new RailMath(this.position2, this.angle2, this.position1, this.angle1, this.shape, this.verticalRadius) : new RailMath(this.position1, this.angle1, this.position2, this.angle2, this.shape, this.verticalRadius);
        this.speedLimit1MetersPerMillisecond = Utilities.kilometersPerHourToMetersPerMillisecond(this.speedLimit1);
        this.speedLimit2MetersPerMillisecond = Utilities.kilometersPerHourToMetersPerMillisecond(this.speedLimit2);
        this.updateData(readerBase);
    }

    @Override
    public boolean isValid() {
        return !(this.speedLimit1 <= 0L && this.speedLimit2 <= 0L || !this.railMath.isValid() || this.isPlatform && this.isSiding);
    }

    @Override
    protected Position getPosition1() {
        return this.position1;
    }

    @Override
    protected Position getPosition2() {
        return this.position2;
    }

    public TransportMode getTransportMode() {
        return this.transportMode;
    }

    public Angle getStartAngle(boolean reversed) {
        return this.reversePositions == reversed ? this.angle1 : this.angle2;
    }

    public Angle getStartAngle(Position startPosition) {
        return this.position1.equals(startPosition) ? this.angle1 : this.angle2;
    }

    public double getSpeedLimitMetersPerMillisecond(boolean reversed) {
        return this.reversePositions == reversed ? this.speedLimit1MetersPerMillisecond : this.speedLimit2MetersPerMillisecond;
    }

    public double getSpeedLimitMetersPerMillisecond(Position startPosition) {
        return this.position1.equals(startPosition) ? this.speedLimit1MetersPerMillisecond : this.speedLimit2MetersPerMillisecond;
    }

    public long getSpeedLimitKilometersPerHour(boolean reversed) {
        return this.reversePositions == reversed ? this.speedLimit1 : this.speedLimit2;
    }

    public boolean canAccelerate() {
        return this.canAccelerate;
    }

    public boolean isPlatform() {
        return this.isPlatform;
    }

    public boolean isSiding() {
        return this.isSiding;
    }

    public boolean canTurnBack() {
        return this.canTurnBack;
    }

    public boolean canConnectRemotely() {
        return this.canConnectRemotely;
    }

    public boolean closeTo(Position position, double radius) {
        return Utilities.isBetween(position, this.railMath.minX, this.railMath.minY, this.railMath.minZ, this.railMath.maxX, this.railMath.maxY, this.railMath.maxZ, radius);
    }

    public void tick1(Simulator simulator) {
        boolean needsUpdate = !Utilities.sameItems(this.preBlockedVehicleIds.keySet(), this.preBlockedVehicleIdsOld.keySet()) || !Utilities.sameItems(this.currentlyBlockedVehicleIds.keySet(), this.currentlyBlockedVehicleIdsOld.keySet());
        simulator.clients.forEach(client -> {
            if (this.closeTo(client.getPosition(), client.getUpdateRadius())) {
                client.update(this, needsUpdate);
            }
        });
        this.preBlockedVehicleIdsOld.clear();
        this.preBlockedVehicleIdsOld.putAll(this.preBlockedVehicleIds);
        this.preBlockedVehicleIds.clear();
        this.currentlyBlockedVehicleIdsOld.clear();
        this.currentlyBlockedVehicleIdsOld.putAll(this.currentlyBlockedVehicleIds);
        this.currentlyBlockedVehicleIds.clear();
    }

    public void tick2(long millisElapsed) {
        if (this.manualBlockCooldown > 0L) {
            if (Rail.isNotBlocked(this.preBlockedVehicleIds, 0L) && Rail.isNotBlocked(this.currentlyBlockedVehicleIds, 0L) && Rail.isNotBlocked(this.preBlockedVehicleIdsOld, 0L) && Rail.isNotBlocked(this.currentlyBlockedVehicleIdsOld, 0L)) {
                this.manualBlockColors.forEach(color -> Rail.reserveRail(0L, color, (ObjectOpenHashSet<Rail>)new ObjectOpenHashSet(), this, true));
            }
            this.manualBlockCooldown = Math.max(0L, this.manualBlockCooldown - millisElapsed);
        }
    }

    public void checkOrCreateSavedRail(Data data, ObjectArrayList<Platform> platformsToAdd, ObjectArrayList<Siding> sidingsToAdd) {
        if (this.isPlatform && data.platforms.stream().noneMatch(platform -> platform.containsPos(this.position1) && platform.containsPos(this.position2))) {
            Platform platform2 = new Platform(this.position1, this.position2, this.transportMode, data);
            data.platforms.add(platform2);
            platformsToAdd.add(platform2);
        }
        if (this.isSiding && data.sidings.stream().noneMatch(siding -> siding.containsPos(this.position1) && siding.containsPos(this.position2))) {
            Siding siding2 = new Siding(this.position1, this.position2, this.railMath.getLength(), this.transportMode, data);
            data.sidings.add(siding2);
            sidingsToAdd.add(siding2);
        }
    }

    public ObjectImmutableList<String> getStyles() {
        return new ObjectImmutableList((ObjectList)this.styles);
    }

    public IntAVLTreeSet getSignalColors() {
        IntAVLTreeSet returnSet = new IntAVLTreeSet();
        this.signalColors.forEach(color -> returnSet.add((int)color));
        return returnSet;
    }

    public void copySignalColors(Rail rail) {
        this.signalColors.clear();
        this.signalColors.addAll((LongList)rail.signalColors);
    }

    public void iteratePreBlockedSignalColors(LongConsumer consumer) {
        this.preBlockedVehicleIds.keySet().forEach(consumer);
    }

    public void iterateCurrentlyBlockedSignalColors(LongConsumer consumer) {
        this.currentlyBlockedVehicleIds.keySet().forEach(consumer);
    }

    public void applyModification(SignalModification signalModification) {
        if (this.matchesPositions(signalModification)) {
            if (signalModification.getIsClearAll()) {
                this.signalColors.clear();
            } else {
                this.signalColors.removeIf(arg_0 -> ((LongArrayList)signalModification.getSignalColorsRemove()).contains(arg_0));
            }
            signalModification.getSignalColorsAdd().forEach(arg_0 -> ((LongArrayList)this.signalColors).add(arg_0));
        }
    }

    public void checkMigrationStatus() {
        if (!this.stylesMigratedLegacy && this.styles.isEmpty()) {
            this.styles.add("default");
            this.stylesMigratedLegacy = true;
        }
    }

    public void blockRail(LongArrayList colors) {
        this.manualBlockCooldown = 1000L;
        this.manualBlockColors.clear();
        this.manualBlockColors.addAll((LongList)(colors.isEmpty() ? this.signalColors : colors));
    }

    boolean isBlocked(long vehicleId, BlockReservation blockReservation) {
        if (this.signalColors.isEmpty() || Rail.isNotBlocked(this.preBlockedVehicleIds, vehicleId) && Rail.isNotBlocked(this.currentlyBlockedVehicleIds, vehicleId) && Rail.isNotBlocked(this.preBlockedVehicleIdsOld, vehicleId) && Rail.isNotBlocked(this.currentlyBlockedVehicleIdsOld, vehicleId)) {
            if (blockReservation != BlockReservation.DO_NOT_RESERVE) {
                this.signalColors.forEach(color -> Rail.reserveRail(vehicleId, color, (ObjectOpenHashSet<Rail>)new ObjectOpenHashSet(), this, blockReservation == BlockReservation.CURRENTLY_RESERVE));
            }
            return false;
        }
        return true;
    }

    void writePositionsToRailCache(Object2ObjectOpenHashMap<Position, Object2ObjectOpenHashMap<Position, Rail>> positionsToRail) {
        Data.put(positionsToRail, this.position1, this.position2, oldValue -> this, Object2ObjectOpenHashMap::new);
        Data.put(positionsToRail, this.position2, this.position1, oldValue -> this, Object2ObjectOpenHashMap::new);
    }

    void writeConnectedRailsCacheFromMap(Object2ObjectOpenHashMap<Position, Object2ObjectOpenHashMap<Position, Rail>> positionsToRail) {
        this.writeConnectedRailsCacheFromMap(positionsToRail, this.position1, this.connectedRails1);
        this.writeConnectedRailsCacheFromMap(positionsToRail, this.position2, this.connectedRails2);
    }

    private void writeConnectedRailsCacheFromMap(Object2ObjectOpenHashMap<Position, Object2ObjectOpenHashMap<Position, Rail>> positionsToRail, Position position, ObjectOpenHashSet<Rail> connectedRails) {
        connectedRails.clear();
        positionsToRail.getOrDefault(position, new Object2ObjectOpenHashMap<>()).forEach((connectedPosition, rail) -> {
            if (!this.equals(rail)) {
                connectedRails.add(rail);
            }
        });
    }

    public static ObjectObjectImmutablePair<Angle, Angle> getAngles(Position positionStart, float angle1, Position positionEnd, float angle2) {
        float angleDifference = (float)Math.toDegrees(Math.atan2(positionEnd.getZ() - positionStart.getZ(), positionEnd.getX() - positionStart.getX()));
        return new ObjectObjectImmutablePair(Angle.fromAngle(angle1 + (float)(Angle.similarFacing(angleDifference, angle1) ? 0 : 180)), Angle.fromAngle(angle2 + (float)(Angle.similarFacing(angleDifference, angle2) ? 180 : 0)));
    }

    private static void reserveRail(long vehicleId, long color, ObjectOpenHashSet<Rail> visitedRails, Rail rail, boolean currentlyBlocked) {
        if (!visitedRails.contains(rail) && rail.signalColors.contains(color)) {
            (currentlyBlocked ? rail.currentlyBlockedVehicleIds : rail.preBlockedVehicleIds).put(color, vehicleId);
            visitedRails.add(rail);
            rail.connectedRails1.forEach(connectedRail -> Rail.reserveRail(vehicleId, color, visitedRails, connectedRail, currentlyBlocked));
            rail.connectedRails2.forEach(connectedRail -> Rail.reserveRail(vehicleId, color, visitedRails, connectedRail, currentlyBlocked));
        }
    }

    private static boolean isNotBlocked(Long2LongAVLTreeMap blockedVehicleIds, long vehicleId) {
        return blockedVehicleIds.values().longStream().allMatch(blockedVehicleId -> blockedVehicleId == vehicleId);
    }

    public static enum BlockReservation {
        DO_NOT_RESERVE,
        PRE_RESERVE,
        CURRENTLY_RESERVE;

    }

    public static enum Shape {
        QUADRATIC,
        TWO_RADII,
        CABLE;

    }
}

