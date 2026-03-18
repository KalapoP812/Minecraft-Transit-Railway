/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanBooleanImmutablePair
 *  org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleDoubleImmutablePair
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2LongAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectOpenHashSet
 */
package org.mtr.core.data;

import java.util.Map;
import javax.annotation.Nullable;
import org.mtr.core.Main;
import org.mtr.core.data.ClientData;
import org.mtr.core.data.Data;
import org.mtr.core.data.Depot;
import org.mtr.core.data.PathData;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.Siding;
import org.mtr.core.data.TransportMode;
import org.mtr.core.data.VehicleCar;
import org.mtr.core.data.VehicleExtraData;
import org.mtr.core.data.VehiclePosition;
import org.mtr.core.data.VehicleRidingEntity;
import org.mtr.core.generated.data.VehicleSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.Utilities;
import org.mtr.core.tool.Vector;
import org.mtr.libraries.it.unimi.dsi.fastutil.booleans.BooleanBooleanImmutablePair;
import org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleDoubleImmutablePair;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2LongAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectOpenHashSet;

public class Vehicle
extends VehicleSchema
implements Utilities {
    private long stoppingCooldown;
    private long deviation;
    private double deviationSpeedAdjustment;
    private long manualCooldown;
    private long doorCooldown;
    private boolean atoOverride;
    public final VehicleExtraData vehicleExtraData;
    private final Siding siding;
    private final boolean isClientside;
    public static final int MAX_POWER_LEVEL = 7;
    public static final int POWER_LEVEL_RATIO = 5;
    public static final int DOOR_MOVE_TIME = 3200;
    private static final int DOOR_DELAY = 1000;

    public Vehicle(VehicleExtraData vehicleExtraData, @Nullable Siding siding, TransportMode transportMode, Data data) {
        super(transportMode, data);
        this.siding = siding;
        this.vehicleExtraData = vehicleExtraData;
        this.isClientside = !(data instanceof Simulator);
    }

    public Vehicle(VehicleExtraData vehicleExtraData, @Nullable Siding siding, ReaderBase readerBase, Data data) {
        super(readerBase, data);
        this.siding = siding;
        this.vehicleExtraData = vehicleExtraData;
        this.isClientside = !(data instanceof Simulator);
        this.updateData(readerBase);
    }

    @Deprecated
    public Vehicle(ReaderBase readerBase) {
        this(new VehicleExtraData(readerBase), null, readerBase, (Data)new ClientData());
    }

    @Override
    public boolean isValid() {
        return true;
    }

    public boolean isMoving() {
        return this.speed != 0.0;
    }

    public boolean getIsOnRoute() {
        return this.railProgress > this.vehicleExtraData.getDefaultPosition();
    }

    public boolean getReversed() {
        return this.reversed;
    }

    public boolean closeToDepot() {
        return !this.getIsOnRoute() || this.railProgress < this.vehicleExtraData.getTotalVehicleLength() + this.vehicleExtraData.getRailLength();
    }

    public void initVehiclePositions(Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>> vehiclePositions) {
        this.writeVehiclePositions(Utilities.getIndexFromConditionalList(this.vehicleExtraData.immutablePath, this.railProgress), vehiclePositions);
    }

    public void simulate(long millisElapsed, @Nullable ObjectArrayList<Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>>> vehiclePositions, @Nullable Long2LongAVLTreeMap vehicleTimesAlongRoute) {
        long timeAlongRoute;
        int currentIndex;
        BooleanBooleanImmutablePair containsDriverAndDoorOverride = this.vehicleExtraData.containsDriverAndDoorOverride();
        this.manualCooldown = this.vehicleExtraData.getIsManualAllowed() && containsDriverAndDoorOverride.leftBoolean() ? this.vehicleExtraData.getManualToAutomaticTime() : Math.max(0L, this.manualCooldown - millisElapsed);
        long l = this.doorCooldown = this.vehicleExtraData.getDoorMultiplier() > 0 || containsDriverAndDoorOverride.rightBoolean() ? 4200L : Math.max(0L, this.doorCooldown - millisElapsed);
        if (this.getIsOnRoute()) {
            if (this.vehicleExtraData.getRepeatIndex2() == 0 && this.railProgress >= this.vehicleExtraData.getTotalDistance() - (this.vehicleExtraData.getRailLength() - this.vehicleExtraData.getTotalVehicleLength()) / 2.0) {
                currentIndex = 0;
                if (!this.isClientside) {
                    this.vehicleExtraData.setPowerLevel(Math.min(this.vehicleExtraData.getPowerLevel(), -1));
                }
                this.simulateInDepot();
            } else {
                currentIndex = Utilities.getIndexFromConditionalList(this.vehicleExtraData.immutablePath, this.railProgress);
                if (this.speed <= 0.0) {
                    this.speed = 0.0;
                    this.simulateStopped(millisElapsed, vehiclePositions, currentIndex);
                } else {
                    this.simulateMoving(millisElapsed, vehiclePositions, currentIndex);
                }
            }
        } else {
            currentIndex = 0;
            this.simulateInDepot();
        }
        this.stoppingCooldown = Math.max(0L, this.stoppingCooldown - millisElapsed);
        if (vehiclePositions != null && vehiclePositions.size() > 1) {
            this.writeVehiclePositions(currentIndex, (Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>>)((Object2ObjectAVLTreeMap)vehiclePositions.get(1)));
        }
        if (vehicleTimesAlongRoute != null && (timeAlongRoute = this.getTimeAlongRoute(this.railProgress)) > 0L) {
            vehicleTimesAlongRoute.put(this.departureIndex, timeAlongRoute);
        }
        if (!this.isClientside) {
            if (this.data instanceof Simulator) {
                this.vehicleExtraData.removeRidingEntitiesIf(vehicleRidingEntity -> !((Simulator)this.data).isRiding(vehicleRidingEntity.uuid, this.id));
            }
            this.vehicleExtraData.setIsCurrentlyManual(this.isCurrentlyManual());
        }
    }

    public void startUp(long newDepartureIndex, long newSidingDepartureTime) {
        if (this.isClientside) {
            Main.LOGGER.warn("Vehicle#startUp should only be called on the server side!");
        }
        this.vehicleExtraData.closeDoors();
        if (this.doorCooldown == 0L) {
            this.departureIndex = newDepartureIndex;
            this.sidingDepartureTime = newSidingDepartureTime;
            this.railProgress += 4.0E-6;
            this.elapsedDwellTime = 0L;
            this.speed = 4.0E-6;
            this.atoOverride = false;
            this.vehicleExtraData.setSpeedTarget(this.speed);
            this.setNextStoppingIndex();
            this.updateDeviation();
            if (this.deviation > 0L && this.nextStoppingIndexAto < (long)(this.vehicleExtraData.immutablePath.size() - 1) && this.siding != null && this.siding.getDelayedVehicleSpeedIncreasePercentage() > 0) {
                double endRailProgress = ((PathData)this.vehicleExtraData.immutablePath.get((int)this.nextStoppingIndexAto)).getEndDistance();
                double distance = endRailProgress - this.railProgress;
                double scheduledDuration = this.getTimeAlongRoute(endRailProgress) - this.getTimeAlongRoute(this.railProgress);
                double expectedDuration = Math.max(1.0, scheduledDuration - (double)this.deviation);
                double averageSpeed = distance / scheduledDuration;
                double expectedSpeed = distance / expectedDuration;
                this.deviationSpeedAdjustment = Math.min(expectedSpeed / averageSpeed, (double)((float)this.siding.getDelayedVehicleSpeedIncreasePercentage() / 100.0f + 1.0f));
            } else {
                this.deviationSpeedAdjustment = 1.0;
            }
        }
    }

    public long getDepartureIndex() {
        return this.departureIndex;
    }

    public ObjectArrayList<ObjectObjectImmutablePair<VehicleCar, ObjectArrayList<ObjectObjectImmutablePair<Vector, Vector>>>> getVehicleCarsAndPositions() {
        ObjectArrayList vehicleCarsAndPositions = new ObjectArrayList();
        double checkRailProgress = this.railProgress - (this.reversed ? this.vehicleExtraData.getTotalVehicleLength() : 0.0);
        for (int i = 0; i < this.vehicleExtraData.immutableVehicleCars.size(); ++i) {
            VehicleCar vehicleCar = (VehicleCar)this.vehicleExtraData.immutableVehicleCars.get(i);
            double halfLength = vehicleCar.getLength() / 2.0;
            ObjectArrayList bogiePositionsList = new ObjectArrayList();
            DoubleArrayList overrideY = new DoubleArrayList();
            bogiePositionsList.add(this.getBogiePositions((checkRailProgress += (double)(this.reversed ? 1 : -1) * vehicleCar.getCouplingPadding1(i == 0)) + (double)(this.reversed ? 1 : -1) * (halfLength + vehicleCar.getBogie1Position()), overrideY));
            if (!vehicleCar.hasOneBogie) {
                bogiePositionsList.add(this.getBogiePositions(checkRailProgress + (double)(this.reversed ? 1 : -1) * (halfLength + vehicleCar.getBogie2Position()), overrideY));
            }
            vehicleCarsAndPositions.add(new ObjectObjectImmutablePair(vehicleCar, bogiePositionsList));
            checkRailProgress += (double)(this.reversed ? 1 : -1) * vehicleCar.getTotalLength(true, false);
        }
        return vehicleCarsAndPositions;
    }

    public Vector getHeadPosition() {
        return this.getPosition(this.railProgress, new DoubleArrayList());
    }

    void updateRidingEntities(ObjectArrayList<VehicleRidingEntity> vehicleRidingEntities) {
        if (!this.isClientside && this.data instanceof Simulator) {
            ObjectOpenHashSet uuidToRemove = new ObjectOpenHashSet();
            ObjectOpenHashSet vehicleRidingEntitiesToAdd = new ObjectOpenHashSet();
            vehicleRidingEntities.forEach(vehicleRidingEntity -> {
                uuidToRemove.add(vehicleRidingEntity.uuid);
                if (vehicleRidingEntity.isOnVehicle()) {
                    vehicleRidingEntitiesToAdd.add(vehicleRidingEntity);
                    ((Simulator)this.data).ride(vehicleRidingEntity.uuid, this.id);
                } else {
                    ((Simulator)this.data).stopRiding(vehicleRidingEntity.uuid);
                }
                if (this.vehicleExtraData.getIsManualAllowed() && vehicleRidingEntity.isDriver()) {
                    int powerLevel = this.vehicleExtraData.getPowerLevel();
                    if (vehicleRidingEntity.manualToggleDoors()) {
                        if (this.speed > 0.0) {
                            this.vehicleExtraData.closeDoors();
                        } else {
                            this.vehicleExtraData.toggleDoors();
                        }
                    }
                    if (vehicleRidingEntity.manualToggleAto()) {
                        boolean bl = this.atoOverride = this.speed > 0.0 && !this.atoOverride;
                    }
                    if (vehicleRidingEntity.manualAccelerate()) {
                        this.vehicleExtraData.setPowerLevel(Math.min(powerLevel + 1, 7));
                        this.atoOverride = false;
                    } else if (vehicleRidingEntity.manualBrake()) {
                        this.vehicleExtraData.setPowerLevel(Math.max(powerLevel - 1, -8));
                        this.atoOverride = false;
                    }
                }
            });
            this.vehicleExtraData.removeRidingEntitiesIf(vehicleRidingEntity -> uuidToRemove.contains(vehicleRidingEntity.uuid));
            this.vehicleExtraData.addRidingEntities((ObjectOpenHashSet<VehicleRidingEntity>)vehicleRidingEntitiesToAdd);
        }
    }

    long getSidingDepartureTime() {
        return this.sidingDepartureTime;
    }

    private void simulateInDepot() {
        this.railProgress = this.vehicleExtraData.getDefaultPosition();
        this.reversed = false;
        this.speed = 0.0;
        this.nextStoppingIndexAto = 0L;
        this.nextStoppingIndexManual = 0L;
        this.departureIndex = -1L;
        this.sidingDepartureTime = -1L;
        this.vehicleExtraData.closeDoors();
        if (!this.isClientside && this.isCurrentlyManual() && this.vehicleExtraData.getPowerLevel() > 0) {
            this.startUp(-1L, this.data.getCurrentMillis());
        }
    }

    private void simulateStopped(long millisElapsed, @Nullable ObjectArrayList<Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>>> vehiclePositions, int currentIndex) {
        if (this.isClientside) {
            return;
        }
        PathData pathData = (PathData)Utilities.getElement(this.vehicleExtraData.immutablePath, currentIndex);
        if (pathData == null) {
            return;
        }
        this.vehicleExtraData.setStoppingPoint(this.railProgress);
        this.stoppingCooldown = 0L;
        if (this.isCurrentlyManual()) {
            if (this.railProgress == pathData.getStartDistance()) {
                boolean isOpposite;
                PathData currentPathData = (PathData)Utilities.getElement(this.vehicleExtraData.immutablePath, currentIndex - 1);
                PathData nextPathData = (PathData)Utilities.getElement(this.vehicleExtraData.immutablePath, this.vehicleExtraData.getRepeatIndex2() > 0 && currentIndex >= this.vehicleExtraData.getRepeatIndex2() ? this.vehicleExtraData.getRepeatIndex1() : currentIndex);
                boolean bl = isOpposite = currentPathData != null && nextPathData != null && currentPathData.isOppositeRail(nextPathData);
                double nextStartDistance = nextPathData == null ? 0.0 : nextPathData.getStartDistance() + (isOpposite ? this.vehicleExtraData.getTotalVehicleLength() : 0.0);
                if (this.vehicleExtraData.getPowerLevel() > 0 && this.railBlockedDistance(currentIndex, nextStartDistance, 0.0, vehiclePositions, true, false) < 0.0) {
                    if (this.doorCooldown == 0L) {
                        this.railProgress = nextStartDistance;
                        if (isOpposite) {
                            this.reversed = !this.reversed;
                        }
                    }
                    this.startUp(this.departureIndex, this.sidingDepartureTime);
                }
            } else if (this.vehicleExtraData.getPowerLevel() > 0 && this.railBlockedDistance(currentIndex, this.railProgress, 0.0, vehiclePositions, true, false) < 0.0) {
                this.startUp(this.departureIndex, this.sidingDepartureTime);
            }
        } else if (this.railProgress == pathData.getStartDistance()) {
            long deviationAdjustment;
            boolean isOpposite;
            PathData currentPathData = (PathData)Utilities.getElement(this.vehicleExtraData.immutablePath, currentIndex - 1);
            PathData nextPathData = (PathData)Utilities.getElement(this.vehicleExtraData.immutablePath, this.vehicleExtraData.getRepeatIndex2() > 0 && currentIndex >= this.vehicleExtraData.getRepeatIndex2() ? this.vehicleExtraData.getRepeatIndex1() : currentIndex);
            boolean bl = isOpposite = currentPathData != null && nextPathData != null && currentPathData.isOppositeRail(nextPathData);
            double nextStartDistance = nextPathData == null ? 0.0 : nextPathData.getStartDistance() + (isOpposite ? this.vehicleExtraData.getTotalVehicleLength() : 0.0);
            long totalDwellMillis = currentPathData == null ? 0L : currentPathData.getDwellTime();
            long doorCloseTime = Math.max(totalDwellMillis / 2L, totalDwellMillis - 3200L - 1000L);
            if (totalDwellMillis > 0L && this.elapsedDwellTime >= 1000L && this.elapsedDwellTime < doorCloseTime) {
                this.vehicleExtraData.openDoors();
            } else if (this.elapsedDwellTime >= doorCloseTime && this.railBlockedDistance(currentIndex, nextStartDistance, 0.0, vehiclePositions, true, false) < 0.0) {
                if (this.doorCooldown == 0L) {
                    this.railProgress = nextStartDistance;
                    if (isOpposite) {
                        this.reversed = !this.reversed;
                    }
                }
                this.startUp(this.departureIndex, this.sidingDepartureTime);
            }
            if (this.siding != null && this.elapsedDwellTime >= 4200L && this.elapsedDwellTime < doorCloseTime && this.deviation != 0L) {
                if (this.deviation > 0L) {
                    deviationAdjustment = Math.min(this.deviation, (doorCloseTime - this.elapsedDwellTime) * (long)this.siding.getDelayedVehicleReduceDwellTimePercentage() / 100L);
                    this.deviation = 0L;
                } else {
                    deviationAdjustment = this.siding.getEarlyVehicleIncreaseDwellTime() ? Math.max(this.deviation, -millisElapsed) : 0L;
                    this.deviation -= deviationAdjustment;
                }
            } else {
                deviationAdjustment = 0L;
            }
            this.elapsedDwellTime = Math.min(this.elapsedDwellTime + millisElapsed + deviationAdjustment, totalDwellMillis);
        } else if (this.railBlockedDistance(currentIndex, this.railProgress, 0.0, vehiclePositions, true, false) < 0.0) {
            this.startUp(this.departureIndex, this.sidingDepartureTime);
        }
    }

    private void simulateMoving(long millisElapsed, @Nullable ObjectArrayList<Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>>> vehiclePositions, int currentIndex) {
        double stoppingDistance;
        int powerLevel;
        double speedTarget;
        double stoppingPoint;
        if (this.isClientside) {
            stoppingPoint = this.vehicleExtraData.getStoppingPoint();
            speedTarget = this.vehicleExtraData.getSpeedTarget();
            powerLevel = this.vehicleExtraData.getPowerLevel();
        } else {
            double safeStoppingDistance = 0.5 * this.speed * this.speed / this.vehicleExtraData.getDeceleration() * (double)(this.isCurrentlyManual() ? 5 : 1);
            double hardStoppingDistance = 0.5 * this.speed * this.speed / 4.0E-5;
            if (this.transportMode.continuousMovement) {
                stoppingPoint = Double.MAX_VALUE;
                if (((PathData)this.vehicleExtraData.immutablePath.get(currentIndex)).getDwellTime() > 0L) {
                    this.vehicleExtraData.openDoors();
                } else {
                    this.vehicleExtraData.closeDoors();
                }
            } else {
                double pathStoppingPoint = this.getPathStoppingPoint();
                if (this.stoppingCooldown > 0L) {
                    stoppingPoint = Math.min(this.vehicleExtraData.getStoppingPoint(), pathStoppingPoint);
                } else {
                    double railBlockedDistance = this.railBlockedDistance(currentIndex, this.railProgress, safeStoppingDistance, vehiclePositions, true, false);
                    if (railBlockedDistance < 0.0) {
                        stoppingPoint = pathStoppingPoint;
                    } else {
                        stoppingPoint = Math.min(railBlockedDistance + this.railProgress, pathStoppingPoint);
                        this.stoppingCooldown = 1000L;
                    }
                }
            }
            double d = this.isCurrentlyManual() ? hardStoppingDistance : safeStoppingDistance;
            if (stoppingPoint - this.railProgress < d) {
                speedTarget = -1.0;
                powerLevel = Math.min(this.vehicleExtraData.getPowerLevel(), this.isCurrentlyManual() && hardStoppingDistance > 2.0 ? -8 : -5);
                this.atoOverride = true;
            } else if (this.isCurrentlyManual()) {
                if (this.speed > this.vehicleExtraData.getMaxManualSpeed()) {
                    speedTarget = this.vehicleExtraData.getMaxManualSpeed();
                    powerLevel = -5;
                } else {
                    powerLevel = this.vehicleExtraData.getPowerLevel();
                    speedTarget = powerLevel > 0 ? this.vehicleExtraData.getMaxManualSpeed() : (powerLevel < 0 ? 0.0 : this.speed);
                }
            } else {
                double upcomingSlowerSpeed = Siding.getUpcomingSlowerSpeed(this.vehicleExtraData.immutablePath, currentIndex, this.railProgress, this.speed, this.vehicleExtraData.getDeceleration());
                if (upcomingSlowerSpeed >= 0.0 && upcomingSlowerSpeed < this.speed) {
                    speedTarget = upcomingSlowerSpeed * this.deviationSpeedAdjustment;
                    powerLevel = -5;
                } else {
                    speedTarget = ((PathData)this.vehicleExtraData.immutablePath.get(currentIndex)).getSpeedLimitMetersPerMillisecond() * this.deviationSpeedAdjustment;
                    powerLevel = Double.compare(speedTarget, this.speed) * 5;
                }
            }
            this.vehicleExtraData.setStoppingPoint(stoppingPoint);
            this.vehicleExtraData.setSpeedTarget(speedTarget);
            this.vehicleExtraData.setPowerLevel(powerLevel);
        }
        this.speed = speedTarget < 0.0 ? ((stoppingDistance = stoppingPoint - this.railProgress) <= 0.0 ? 4.0E-6 : Math.max(this.speed - 0.5 * this.speed * this.speed / stoppingDistance * (double)millisElapsed, 4.0E-6)) : (powerLevel > 0 ? Math.min(this.speed + this.vehicleExtraData.getAcceleration() * (double)powerLevel / 5.0 * (double)millisElapsed, speedTarget) : (powerLevel < 0 ? Math.max(this.speed + (powerLevel < -7 ? -4.0E-5 : this.vehicleExtraData.getDeceleration() * (double)powerLevel / 5.0) * (double)millisElapsed, speedTarget) : speedTarget));
        this.railProgress += this.speed * (double)millisElapsed;
        if (this.railProgress >= stoppingPoint) {
            this.railProgress = stoppingPoint;
            this.speed = 0.0;
            this.vehicleExtraData.setSpeedTarget(0.0);
            this.updateDeviation();
            if (!this.isClientside) {
                this.atoOverride = false;
                this.vehicleExtraData.setPowerLevel(Math.min(this.vehicleExtraData.getPowerLevel(), -1));
            }
        } else if (this.vehicleExtraData.getRepeatIndex2() > 0 && this.railProgress >= this.vehicleExtraData.getTotalDistance()) {
            this.railProgress = ((PathData)this.vehicleExtraData.immutablePath.get(this.vehicleExtraData.getRepeatIndex1())).getStartDistance() + this.railProgress - this.vehicleExtraData.getTotalDistance();
        }
    }

    private double getPathStoppingPoint() {
        PathData pathDataAto = (PathData)Utilities.getElement(this.vehicleExtraData.immutablePath, (int)this.nextStoppingIndexAto);
        boolean pastAtoStoppingPoint = pathDataAto != null && this.railProgress > pathDataAto.getEndDistance();
        int nextStoppingIndex = (int)(this.isCurrentlyManual() || pastAtoStoppingPoint ? this.nextStoppingIndexManual : this.nextStoppingIndexAto);
        double stoppingPointByStoppingIndex = nextStoppingIndex >= this.vehicleExtraData.immutablePath.size() - 1 ? this.vehicleExtraData.getTotalDistance() - (this.vehicleExtraData.getRepeatIndex2() > 0 ? 0.0 : (this.vehicleExtraData.getRailLength() - this.vehicleExtraData.getTotalVehicleLength()) / 2.0) : ((PathData)this.vehicleExtraData.immutablePath.get(nextStoppingIndex)).getEndDistance();
        if (pastAtoStoppingPoint) {
            this.setNextStoppingIndex();
        }
        return stoppingPointByStoppingIndex;
    }

    private boolean isCurrentlyManual() {
        if (this.isClientside) {
            Main.LOGGER.warn("Vehicle#isCurrentlyManual should only be called on the server side!");
        }
        return !this.atoOverride && this.manualCooldown > 0L;
    }

    private void setNextStoppingIndex() {
        this.nextStoppingIndexAto = this.nextStoppingIndexManual = (long)(this.vehicleExtraData.immutablePath.size() - 1);
        this.vehicleExtraData.setStoppingPoint(this.vehicleExtraData.getTotalDistance());
        for (int i = Utilities.getIndexFromConditionalList(this.vehicleExtraData.immutablePath, this.railProgress); i < this.vehicleExtraData.immutablePath.size(); ++i) {
            PathData pathData = (PathData)this.vehicleExtraData.immutablePath.get(i);
            if (pathData.getDwellTime() <= 0L) continue;
            if (this.vehicleExtraData.getIsManualAllowed()) {
                this.nextStoppingIndexAto = Math.min(this.nextStoppingIndexAto, (long)i);
                if (i >= this.vehicleExtraData.immutablePath.size() - 1 || !((PathData)this.vehicleExtraData.immutablePath.get(i + 1)).isOppositeRail(pathData)) continue;
                this.nextStoppingIndexManual = i;
                this.vehicleExtraData.setStoppingPoint(pathData.getEndDistance());
                break;
            }
            this.nextStoppingIndexAto = this.nextStoppingIndexManual = (long)i;
            this.vehicleExtraData.setStoppingPoint(pathData.getEndDistance());
            break;
        }
    }

    private void writeVehiclePositions(int currentIndex, Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>> vehiclePositions) {
        int index;
        Position[] minMaxPositions = new Position[]{null, null};
        for (index = currentIndex; index >= 0; --index) {
            DoubleDoubleImmutablePair blockedBounds;
            PathData pathData = (PathData)this.vehicleExtraData.immutablePath.get(index);
            Position position1 = pathData.getOrderedPosition1();
            Position position2 = pathData.getOrderedPosition2();
            minMaxPositions[0] = Position.getMin(minMaxPositions[0], Position.getMin(position1, position2));
            minMaxPositions[1] = Position.getMax(minMaxPositions[1], Position.getMax(position1, position2));
            if (this.railProgress - this.vehicleExtraData.getTotalVehicleLength() > pathData.getEndDistance()) break;
            if (this.transportMode.continuousMovement || !((blockedBounds = Vehicle.getBlockedBounds(pathData, this.railProgress - this.vehicleExtraData.getTotalVehicleLength(), this.railProgress - 0.01)).rightDouble() - blockedBounds.leftDouble() > 0.01) || !this.getIsOnRoute() || index <= 0) continue;
            Data.put(vehiclePositions, position1, position2, vehiclePosition -> {
                VehiclePosition newVehiclePosition = vehiclePosition == null ? new VehiclePosition() : vehiclePosition;
                newVehiclePosition.addSegment(blockedBounds.leftDouble(), blockedBounds.rightDouble(), this.id);
                return newVehiclePosition;
            }, Object2ObjectAVLTreeMap::new);
            pathData.isSignalBlocked(this.id, Rail.BlockReservation.CURRENTLY_RESERVE);
        }
        if (this.siding != null) {
            if (this.siding.area != null && this.data instanceof Simulator) {
                boolean needsUpdate = this.vehicleExtraData.checkForUpdate();
                int pathUpdateIndex = this.transportMode.continuousMovement ? 0 : Math.max(0, index + 1);
                ((Simulator)this.data).clients.forEach(client -> {
                    Position position = client.getPosition();
                    double updateRadius = client.getUpdateRadius();
                    if (minMaxPositions[0] == null || minMaxPositions[1] == null ? ((Depot)this.siding.area).inArea(position, updateRadius) : Utilities.isBetween(position, minMaxPositions[0], minMaxPositions[1], updateRadius) || !this.closeToDepot() && this.vehicleExtraData.hasRidingEntity(client.uuid)) {
                        client.update(this, needsUpdate, pathUpdateIndex);
                    }
                });
            }
            this.vehicleExtraData.setRoutePlatformInfo((Depot)this.siding.area, currentIndex);
        }
    }

    private double railBlockedDistance(int currentIndex, double checkRailProgress, double checkDistance, @Nullable ObjectArrayList<Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>>> vehiclePositions, boolean reserveRail, boolean secondPass) {
        for (int index = currentIndex; vehiclePositions != null && index < this.vehicleExtraData.immutablePath.size(); ++index) {
            PathData pathData = (PathData)this.vehicleExtraData.immutablePath.get(index);
            double checkRailProgressEnd = checkRailProgress + checkDistance + (double)this.transportMode.stoppingSpace;
            if (pathData.getStartDistance() >= checkRailProgressEnd) {
                return -1.0;
            }
            double blockedStartOffset = Math.max(0.0, pathData.getStartDistance() - checkRailProgress);
            if (this.checkAndBlockSignal(index, vehiclePositions, reserveRail, secondPass)) {
                return blockedStartOffset;
            }
            if (!Utilities.isIntersecting(pathData.getStartDistance(), pathData.getEndDistance(), checkRailProgress, checkRailProgressEnd)) continue;
            DoubleDoubleImmutablePair blockedBounds = Vehicle.getBlockedBounds(pathData, checkRailProgress, checkRailProgressEnd);
            for (int i = 0; i < 2; ++i) {
                double closestOverlap;
                VehiclePosition vehiclePosition = (VehiclePosition)Data.tryGet(vehiclePositions.get(i), pathData.getOrderedPosition1(), pathData.getOrderedPosition2());
                if (vehiclePosition == null || !((closestOverlap = vehiclePosition.getClosestOverlap(blockedBounds.leftDouble(), blockedBounds.rightDouble(), pathData.reversePositions, this.id)) >= 0.0)) continue;
                return Math.max(0.0, blockedStartOffset + closestOverlap - (double)this.transportMode.stoppingSpace);
            }
        }
        return -1.0;
    }

    private boolean checkAndBlockSignal(int currentIndex, ObjectArrayList<Object2ObjectAVLTreeMap<Position, Object2ObjectAVLTreeMap<Position, VehiclePosition>>> vehiclePositions, boolean reserveRail, boolean secondPass) {
        PathData firstPathData = (PathData)this.vehicleExtraData.immutablePath.get(currentIndex);
        if (secondPass) {
            return firstPathData.isSignalBlocked(this.id, Rail.BlockReservation.DO_NOT_RESERVE);
        }
        IntAVLTreeSet signalColors = firstPathData.getSignalColors();
        for (int index = currentIndex + 1; !signalColors.isEmpty() && index < this.vehicleExtraData.immutablePath.size(); ++index) {
            PathData pathData = (PathData)this.vehicleExtraData.immutablePath.get(index);
            if (!pathData.getSignalColors().intStream().noneMatch(arg_0 -> ((IntAVLTreeSet)signalColors).contains(arg_0))) continue;
            double railBlockedDistance = this.railBlockedDistance(index, pathData.getStartDistance(), this.vehicleExtraData.getTotalVehicleLength(), vehiclePositions, false, true);
            return railBlockedDistance >= 0.0 && railBlockedDistance < this.vehicleExtraData.getTotalVehicleLength() || firstPathData.isSignalBlocked(this.id, reserveRail ? Rail.BlockReservation.PRE_RESERVE : Rail.BlockReservation.DO_NOT_RESERVE);
        }
        return false;
    }

    private long getTimeAlongRoute(double checkRailProgress) {
        return this.siding == null ? 0L : (long)Math.floor(this.siding.getTimeAlongRoute(checkRailProgress - (double)(this.speed == 0.0 ? 1 : 0)) + (double)this.elapsedDwellTime);
    }

    private void updateDeviation() {
        this.deviation = this.transportMode.continuousMovement || this.siding == null ? 0L : Utilities.circularDifference(this.data.getCurrentMillis() - this.sidingDepartureTime, this.getTimeAlongRoute(this.railProgress), this.siding.getRepeatInterval(86400000L));
    }

    @Nullable
    private Vector getPosition(double value, DoubleArrayList overrideY) {
        PathData pathData = (PathData)Utilities.getElement(this.vehicleExtraData.immutablePath, Utilities.getIndexFromConditionalList(this.vehicleExtraData.immutablePath, value));
        if (pathData == null) {
            return null;
        }
        Vector vector = pathData.getPosition(value - pathData.getStartDistance());
        if (this.transportMode == TransportMode.AIRPLANE && pathData.getSpeedLimitKilometersPerHour() == 300L && pathData.isDescending()) {
            if (overrideY.isEmpty()) {
                overrideY.add(vector.y());
                return vector;
            }
            return new Vector(vector.x(), overrideY.getDouble(0), vector.z());
        }
        return vector;
    }

    private ObjectObjectImmutablePair<Vector, Vector> getBogiePositions(double value, DoubleArrayList overrideY) {
        double lowerBound = this.railProgress - this.vehicleExtraData.getTotalVehicleLength();
        double clampedValue = Utilities.clamp(value, lowerBound, this.railProgress);
        double clamp = Utilities.clamp(Math.min(Math.abs(clampedValue - lowerBound), Math.abs(clampedValue - this.railProgress)), 0.1, 1.0);
        double value1 = Utilities.clamp(clampedValue + (this.reversed ? -clamp : clamp), lowerBound, this.railProgress - 0.001);
        double value2 = Utilities.clamp(clampedValue - (this.reversed ? -clamp : clamp), lowerBound, this.railProgress - 0.001);
        Vector position1 = this.getPosition(value1, overrideY);
        Vector position2 = this.getPosition(value2, overrideY);
        return position1 == null || position2 == null ? new ObjectObjectImmutablePair(new Vector(value1, 0.0, 0.0), new Vector(value2, 0.0, 0.0)) : new ObjectObjectImmutablePair(position1, position2);
    }

    private static DoubleDoubleImmutablePair getBlockedBounds(PathData pathData, double lowerRailProgress, double upperRailProgress) {
        double distanceFromStart = Utilities.clamp(lowerRailProgress, pathData.getStartDistance(), pathData.getEndDistance()) - pathData.getStartDistance();
        double distanceToEnd = pathData.getEndDistance() - Utilities.clamp(upperRailProgress, pathData.getStartDistance(), pathData.getEndDistance());
        return new DoubleDoubleImmutablePair(pathData.reversePositions ? distanceToEnd : distanceFromStart, pathData.getEndDistance() - pathData.getStartDistance() - (pathData.reversePositions ? distanceFromStart : distanceToEnd));
    }
}

