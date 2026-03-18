/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.com.google.gson.JsonElement
 */
package org.mtr.core.servlet;

import javax.annotation.Nullable;
import org.mtr.core.operation.ArrivalsRequest;
import org.mtr.core.operation.BlockRails;
import org.mtr.core.operation.DataRequest;
import org.mtr.core.operation.DeleteDataRequest;
import org.mtr.core.operation.DepotOperationByIds;
import org.mtr.core.operation.DepotOperationByName;
import org.mtr.core.operation.GenerateByLift;
import org.mtr.core.operation.ListDataResponse;
import org.mtr.core.operation.NearbyAreasRequest;
import org.mtr.core.operation.PressLift;
import org.mtr.core.operation.RailsRequest;
import org.mtr.core.operation.SetTime;
import org.mtr.core.operation.UpdateDataRequest;
import org.mtr.core.operation.UpdateVehicleRidingEntities;
import org.mtr.core.serializer.JsonReader;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.com.google.gson.JsonElement;

public final class OperationProcessor {
    public static final String GET_DATA = "get_data";
    public static final String UPDATE_DATA = "update_data";
    public static final String DELETE_DATA = "delete_data";
    public static final String LIST_DATA = "list_data";
    public static final String ARRIVALS = "arrivals";
    public static final String SET_TIME = "set_time";
    public static final String UPDATE_RIDING_ENTITIES = "update_riding_entities";
    public static final String BLOCK_RAILS = "block_rails";
    public static final String PRESS_LIFT = "press_lift";
    public static final String NEARBY_STATIONS = "nearby_stations";
    public static final String NEARBY_DEPOTS = "nearby_depots";
    public static final String RAILS = "rails";
    public static final String GENERATE_BY_DEPOT_IDS = "generate_by_depot_ids";
    public static final String GENERATE_BY_DEPOT_NAME = "generate_by_depot_name";
    public static final String GENERATE_BY_LIFT = "generate_by_lift";
    public static final String CLEAR_BY_DEPOT_IDS = "clear_by_depot_ids";
    public static final String CLEAR_BY_DEPOT_NAME = "clear_by_depot_name";
    public static final String INSTANT_DEPLOY_BY_DEPOT_IDS = "instant_deploy_by_depot_ids";
    public static final String INSTANT_DEPLOY_BY_DEPOT_NAME = "instant_deploy_by_depot_name";
    public static final String VEHICLES_LIFTS = "vehicles_lifts";
    public static final String GENERATION_STATUS_UPDATE = "generation_status_update";

    @Nullable
    public static SerializedDataBase process(String key, SerializedDataBase data, Simulator simulator) {
        JsonReader jsonReader = new JsonReader((JsonElement)Utilities.getJsonObjectFromData(data));
        switch (key) {
            case "get_data": {
                return new DataRequest(jsonReader).getData(simulator);
            }
            case "update_data": {
                return new UpdateDataRequest(jsonReader, simulator).update();
            }
            case "delete_data": {
                return new DeleteDataRequest(jsonReader).delete(simulator);
            }
            case "list_data": {
                return new ListDataResponse(jsonReader, simulator).list();
            }
            case "arrivals": {
                return new ArrivalsRequest(jsonReader).getArrivals(simulator);
            }
            case "set_time": {
                new SetTime(jsonReader).setGameTime(simulator);
                return null;
            }
            case "update_riding_entities": {
                return new UpdateVehicleRidingEntities(jsonReader).update(simulator);
            }
            case "block_rails": {
                new BlockRails(jsonReader).blockRails(simulator);
                return null;
            }
            case "press_lift": {
                new PressLift(jsonReader).pressLift(simulator);
                return null;
            }
            case "nearby_stations": {
                return new NearbyAreasRequest(jsonReader).query(simulator, simulator.stations);
            }
            case "nearby_depots": {
                return new NearbyAreasRequest(jsonReader).query(simulator, simulator.depots);
            }
            case "rails": {
                return new RailsRequest(jsonReader).query(simulator);
            }
            case "generate_by_depot_ids": {
                new DepotOperationByIds(jsonReader).generate(simulator);
                return null;
            }
            case "generate_by_depot_name": {
                new DepotOperationByName(jsonReader).generate(simulator);
                return null;
            }
            case "generate_by_lift": {
                new GenerateByLift(jsonReader, simulator).generate();
                return null;
            }
            case "clear_by_depot_ids": {
                new DepotOperationByIds(jsonReader).clear(simulator);
                return null;
            }
            case "clear_by_depot_name": {
                new DepotOperationByName(jsonReader).clear(simulator);
                return null;
            }
            case "instant_deploy_by_depot_ids": {
                new DepotOperationByIds(jsonReader).instantDeploy(simulator);
                return null;
            }
            case "instant_deploy_by_depot_name": {
                new DepotOperationByName(jsonReader).instantDeploy(simulator);
                return null;
            }
        }
        return null;
    }
}

