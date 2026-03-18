/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet
 */
package org.mtr.core.data;

import java.util.Map;
import java.util.Set;
import java.util.UUID;
import java.util.function.Consumer;
import org.mtr.core.data.Data;
import org.mtr.core.data.Lift;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.Vehicle;
import org.mtr.core.generated.data.ClientSchema;
import org.mtr.core.operation.PlayerPresentResponse;
import org.mtr.core.operation.VehicleLiftResponse;
import org.mtr.core.operation.VehicleUpdate;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet;

public class Client
extends ClientSchema {
    public final UUID uuid;
    private final LongAVLTreeSet existingVehicleIds = new LongAVLTreeSet();
    private final LongAVLTreeSet keepVehicleIds = new LongAVLTreeSet();
    private final Long2ObjectAVLTreeMap<VehicleUpdate> vehicleUpdates = new Long2ObjectAVLTreeMap();
    private final LongAVLTreeSet existingLiftIds = new LongAVLTreeSet();
    private final LongAVLTreeSet keepLiftIds = new LongAVLTreeSet();
    private final Long2ObjectAVLTreeMap<Lift> liftUpdates = new Long2ObjectAVLTreeMap();
    private final ObjectAVLTreeSet<String> existingRailIds = new ObjectAVLTreeSet();
    private final ObjectAVLTreeSet<String> keepRailIds = new ObjectAVLTreeSet();
    private final Object2ObjectAVLTreeMap<String, Rail> signalBlockUpdates = new Object2ObjectAVLTreeMap();

    public Client(UUID uuid) {
        super(uuid.toString());
        this.uuid = uuid;
    }

    public Client(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
        this.uuid = UUID.fromString(this.clientId);
    }

    @Override
    protected Position getDefaultPosition() {
        return new Position(0L, 0L, 0L);
    }

    public Position getPosition() {
        return this.position;
    }

    public double getUpdateRadius() {
        return this.updateRadius;
    }

    public void setPositionAndUpdateRadius(Position position, long updateRadius) {
        this.position = position;
        this.updateRadius = updateRadius;
    }

    public void sendUpdates(Simulator simulator) {
        VehicleLiftResponse vehicleLiftResponse = new VehicleLiftResponse(this.uuid, (Data)simulator);
        boolean hasUpdate1 = Client.process(this.vehicleUpdates, this.existingVehicleIds, this.keepVehicleIds, vehicleLiftResponse::addVehicleToUpdate, vehicleLiftResponse::addVehicleToKeep);
        boolean hasUpdate2 = Client.process(this.liftUpdates, this.existingLiftIds, this.keepLiftIds, vehicleLiftResponse::addLiftToUpdate, vehicleLiftResponse::addLiftToKeep);
        boolean hasUpdate3 = Client.process(this.signalBlockUpdates, this.existingRailIds, this.keepRailIds, vehicleLiftResponse::addSignalBlockUpdate, railId -> {});
        if (hasUpdate1 || hasUpdate2 || hasUpdate3) {
            simulator.sendMessageS2C("vehicles_lifts", vehicleLiftResponse, playerPresentResponse -> playerPresentResponse.verify(simulator, this.uuid), PlayerPresentResponse.class);
        }
    }

    public void update(Vehicle vehicle, boolean needsUpdate, int pathUpdateIndex) {
        long vehicleId = vehicle.getId();
        if (needsUpdate || !this.existingVehicleIds.contains(vehicleId)) {
            this.vehicleUpdates.put(vehicleId, new VehicleUpdate(vehicle, vehicle.vehicleExtraData.copy(pathUpdateIndex)));
            this.keepVehicleIds.remove(vehicleId);
        } else if (!this.vehicleUpdates.containsKey(vehicleId)) {
            this.keepVehicleIds.add(vehicleId);
        }
    }

    public void update(Lift lift, boolean needsUpdate) {
        long liftId = lift.getId();
        if (needsUpdate || !this.existingLiftIds.contains(liftId)) {
            this.liftUpdates.put(liftId, lift);
            this.keepLiftIds.remove(liftId);
        } else if (!this.liftUpdates.containsKey(liftId)) {
            this.keepLiftIds.add(liftId);
        }
    }

    public void update(Rail rail, boolean needsUpdate) {
        String railId = rail.getHexId();
        if (needsUpdate || !this.existingRailIds.contains(railId)) {
            this.signalBlockUpdates.put(railId, rail);
            this.keepRailIds.remove(railId);
        } else if (!this.signalBlockUpdates.containsKey(railId)) {
            this.keepRailIds.add(railId);
        }
    }

    private static <T, U extends SerializedDataBase> boolean process(Map<T, U> dataUpdates, Set<T> existingIds, Set<T> keepIds, Consumer<U> addDataToUpdate, Consumer<T> addDataToKeep) {
        dataUpdates.forEach((id, data) -> {
            addDataToUpdate.accept(data);
            existingIds.remove(id);
        });
        keepIds.forEach(id -> {
            addDataToKeep.accept(id);
            existingIds.remove(id);
        });
        boolean hasUpdate = !existingIds.isEmpty() || !dataUpdates.isEmpty();
        existingIds.clear();
        existingIds.addAll(dataUpdates.keySet());
        existingIds.addAll(keepIds);
        dataUpdates.clear();
        keepIds.clear();
        return hasUpdate;
    }
}

