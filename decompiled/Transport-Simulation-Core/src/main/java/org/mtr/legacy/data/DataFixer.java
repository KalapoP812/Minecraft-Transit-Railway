/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectArrayMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.org.msgpack.core.MessageBufferPacker
 *  org.mtr.libraries.org.msgpack.core.MessagePack
 *  org.mtr.libraries.org.msgpack.core.MessagePacker
 *  org.mtr.libraries.org.msgpack.core.MessageUnpacker
 *  org.mtr.libraries.org.msgpack.value.Value
 */
package org.mtr.legacy.data;

import java.util.Collection;
import java.util.Locale;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import org.mtr.core.Main;
import org.mtr.core.data.LiftFloor;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.TransportMode;
import org.mtr.core.data.VehicleCar;
import org.mtr.core.serializer.MessagePackReader;
import org.mtr.core.serializer.MessagePackWriter;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.serializer.WriterBase;
import org.mtr.core.tool.Angle;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectArrayMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.org.msgpack.core.MessageBufferPacker;
import org.mtr.libraries.org.msgpack.core.MessagePack;
import org.mtr.libraries.org.msgpack.core.MessagePacker;
import org.mtr.libraries.org.msgpack.core.MessageUnpacker;
import org.mtr.libraries.org.msgpack.value.Value;

public final class DataFixer {
    private static final int PACKED_X_LENGTH = 26;
    private static final int PACKED_Z_LENGTH = 26;
    private static final int PACKED_Y_LENGTH = 12;
    private static final int Z_OFFSET = 12;
    private static final int X_OFFSET = 38;
    private static final int SIZE_BITS_X;
    private static final int SIZE_BITS_Z;
    private static final int SIZE_BITS_Y;
    private static final int BIT_SHIFT_Z;
    private static final int BIT_SHIFT_X;
    private static final long BITS_X;
    private static final long BITS_Y;
    private static final long BITS_Z;

    public static void unpackAreaBasePositions(ReaderBase readerBase, BiConsumer<Position, Position> consumer) {
        readerBase.unpackInt("x_min", xMin -> readerBase.unpackInt("z_min", zMin -> readerBase.unpackInt("x_max", xMax -> readerBase.unpackInt("z_max", zMax -> consumer.accept(new Position(xMin, Long.MIN_VALUE, zMin), new Position(xMax, Long.MAX_VALUE, zMax))))));
    }

    public static void unpackDepotDepartures(ReaderBase readerBase, LongArrayList realTimeDepartures) {
        readerBase.iterateIntArray("departures", () -> ((LongArrayList)realTimeDepartures).clear(), arg_0 -> ((LongArrayList)realTimeDepartures).add(arg_0));
    }

    public static void unpackPlatformDwellTime(ReaderBase readerBase, IntConsumer consumer) {
        readerBase.unpackInt("dwell_time", value -> consumer.accept(value * 500));
    }

    public static ReaderBase convertRoute(ReaderBase readerBase) {
        DataFixer.packExtra(readerBase, messagePackWriter -> {
            LongArrayList platformIds = new LongArrayList();
            readerBase.iterateLongArray("platform_ids", () -> ((LongArrayList)platformIds).clear(), arg_0 -> ((LongArrayList)platformIds).add(arg_0));
            if (!platformIds.isEmpty()) {
                ObjectArrayList customDestinations = new ObjectArrayList();
                readerBase.iterateStringArray("custom_destinations", () -> ((ObjectArrayList)customDestinations).clear(), arg_0 -> ((ObjectArrayList)customDestinations).add(arg_0));
                WriterBase.Array arrayWriter = messagePackWriter.writeArray("routePlatformData");
                for (int i = 0; i < platformIds.size(); ++i) {
                    WriterBase childWriter = arrayWriter.writeChild();
                    childWriter.writeLong("platformId", platformIds.getLong(i));
                    if (i >= customDestinations.size()) continue;
                    childWriter.writeString("customDestination", (String)customDestinations.get(i));
                }
            }
            boolean[] isLightRailRoute = new boolean[]{false};
            readerBase.unpackBoolean("is_light_rail_route", value -> {
                isLightRailRoute[0] = value;
            });
            readerBase.unpackString("light_rail_route_number", value -> messagePackWriter.writeString("routeNumber", isLightRailRoute[0] ? value : ""));
            readerBase.unpackBoolean("is_route_hidden", value -> messagePackWriter.writeBoolean("hidden", value));
        });
        return readerBase;
    }

    public static ReaderBase convertSavedRailBase(ReaderBase readerBase) {
        DataFixer.packExtra(readerBase, messagePackWriter -> {
            readerBase.unpackLong("pos_1", value -> DataFixer.convertPosition(value).serializeData(messagePackWriter.writeChild("position1")));
            readerBase.unpackLong("pos_2", value -> DataFixer.convertPosition(value).serializeData(messagePackWriter.writeChild("position2")));
        });
        return readerBase;
    }

    public static ReaderBase convertSiding(ReaderBase readerBase) {
        DataFixer.packExtra(readerBase, messagePackWriter -> {
            readerBase.unpackInt("dwell_time", value -> messagePackWriter.writeLong("manualToAutomaticTime", (long)value * 500L));
            readerBase.unpackDouble("acceleration_constant", value -> {
                messagePackWriter.writeDouble("acceleration", value / 50.0 / 50.0);
                messagePackWriter.writeDouble("deceleration", value / 50.0 / 50.0);
            });
            readerBase.unpackInt("max_manual_speed", value -> {
                if (value >= 0 && value <= RailType.DIAMOND.ordinal()) {
                    messagePackWriter.writeDouble("maxManualSpeed", RailType.values()[value].speedLimitMetersPerMillisecond);
                }
            });
        });
        return readerBase;
    }

    public static void unpackSidingVehicleCars(ReaderBase readerBase, TransportMode transportMode, double railLength, ObjectArrayList<VehicleCar> vehicleCars) {
        readerBase.unpackString("train_type", baseTrainType -> readerBase.unpackString("train_custom_id", trainId -> {
            try {
                String newBaseTrainType = baseTrainType;
                try {
                    newBaseTrainType = Enum.valueOf(TrainType.class, baseTrainType.toUpperCase(Locale.ENGLISH)).baseTrainType;
                }
                catch (Exception exception) {
                    // empty catch block
                }
                String[] trainTypeSplit = newBaseTrainType.split("_");
                int trainLength = Integer.parseInt(trainTypeSplit[trainTypeSplit.length - 2]) + 1;
                int trainWidth = Integer.parseInt(trainTypeSplit[trainTypeSplit.length - 1]);
                int trainCars = Math.min(transportMode.maxLength, (int)Math.floor(railLength / (double)trainLength));
                double bogiePosition = trainLength < 10 ? 0.0 : (double)trainLength * 0.34;
                for (int i = 0; i < trainCars; ++i) {
                    int type = trainCars == 1 ? 3 : (i == 0 ? 1 : (i == trainCars - 1 ? 2 : 0));
                    String vehicleId = transportMode == TransportMode.TRAIN ? (trainId.startsWith("light_rail") ? trainId + (trainId.endsWith("_rht") ? "" : "_lht") : trainId + (type == 0 ? "_trailer" : "_cab_" + type)) : (transportMode == TransportMode.CABLE_CAR ? trainId + (trainId.endsWith("_rht") ? "" : "_lht") : (transportMode == TransportMode.BOAT ? trainId + "_small" : trainId));
                    vehicleCars.add(new VehicleCar(vehicleId.toLowerCase(Locale.ENGLISH), trainLength, trainWidth, -bogiePosition, bogiePosition, (type & 1) == 0 ? 0.0 : 1.0, (type & 2) == 0 ? 0.0 : 1.0));
                }
            }
            catch (Exception exception) {
                // empty catch block
            }
        }));
    }

    public static void unpackSidingMaxVehicles(ReaderBase readerBase, IntConsumer consumer) {
        readerBase.unpackBoolean("is_manual", value1 -> {
            if (value1) {
                consumer.accept(-1);
            } else {
                readerBase.unpackBoolean("unlimited_trains", value2 -> {
                    if (value2) {
                        consumer.accept(0);
                    } else {
                        readerBase.unpackInt("max_trains", value3 -> consumer.accept(value3 + 1));
                    }
                });
            }
        });
    }

    public static ReaderBase convertStation(ReaderBase readerBase) {
        DataFixer.packExtra(readerBase, messagePackWriter -> {
            readerBase.unpackInt("zone", value -> messagePackWriter.writeLong("zone1", value));
            if (readerBase instanceof MessagePackReader) {
                try {
                    Object2ObjectArrayMap<String, ObjectArrayList<String>> exits = new Object2ObjectArrayMap<>();
                    ((MessagePackReader)readerBase).iterateMap("exits", (key, value) -> {
                        ObjectArrayList<String> destinations = new ObjectArrayList<>();
                        exits.put(key, destinations);
                        value.asArrayValue().forEach(destination -> destinations.add(destination.asStringValue().asString()));
                    });
                    WriterBase.Array exitsWriterBaseArray = messagePackWriter.writeArray("exits");
                    exits.forEach((name, destinations) -> {
                        WriterBase writerBase = exitsWriterBaseArray.writeChild();
                        writerBase.writeString("name", name);
                        WriterBase.Array destinationsWriterBaseArray = writerBase.writeArray("destinations");
                        destinations.forEach(destinationsWriterBaseArray::writeString);
                    });
                }
                catch (Exception exception) {
                    // empty catch block
                }
            }
        });
        return readerBase;
    }

    public static ReaderBase convertLift(ReaderBase readerBase) {
        DataFixer.packExtra(readerBase, messagePackWriter -> readerBase.unpackInt("lift_height", value -> messagePackWriter.writeDouble("height", (double)value / 2.0)));
        DataFixer.packExtra(readerBase, messagePackWriter -> readerBase.unpackInt("lift_width", value -> messagePackWriter.writeDouble("width", value)));
        DataFixer.packExtra(readerBase, messagePackWriter -> readerBase.unpackInt("lift_depth", value -> messagePackWriter.writeDouble("depth", value)));
        DataFixer.packExtra(readerBase, messagePackWriter -> readerBase.unpackInt("lift_offset_x", value -> messagePackWriter.writeDouble("offsetX", (double)value / 2.0)));
        DataFixer.packExtra(readerBase, messagePackWriter -> readerBase.unpackInt("lift_offset_y", value -> messagePackWriter.writeDouble("offsetY", value)));
        DataFixer.packExtra(readerBase, messagePackWriter -> readerBase.unpackInt("lift_offset_z", value -> messagePackWriter.writeDouble("offsetZ", (double)value / 2.0)));
        DataFixer.packExtra(readerBase, messagePackWriter -> readerBase.unpackString("lift_style", value -> messagePackWriter.writeString("style", (String)value)));
        DataFixer.packExtra(readerBase, messagePackWriter -> readerBase.unpackInt("facing", value -> {
            messagePackWriter.writeString("angle", Angle.fromAngle(value - 90).toString());
            ObjectArrayList liftFloors = new ObjectArrayList();
            readerBase.iterateLongArray("floors", () -> {}, floor -> liftFloors.add(new LiftFloor(DataFixer.convertPosition(floor))));
            messagePackWriter.writeDataset((Collection<? extends SerializedDataBase>)liftFloors, "floors");
        }));
        return readerBase;
    }

    public static void readerBaseConvertKey(String key, Value value, Object2ObjectArrayMap<String, Value> map) {
        map.put(key, value);
        String[] keySplit = key.split("_");
        if (keySplit.length == 0) {
            return;
        }
        StringBuilder stringBuilder = new StringBuilder(keySplit[0]);
        for (int i = 1; i < keySplit.length; ++i) {
            String keyPart = keySplit[i];
            if (keyPart.isEmpty()) continue;
            stringBuilder.append(keyPart.substring(0, 1).toUpperCase(Locale.ENGLISH));
            stringBuilder.append(keyPart.substring(1));
        }
        map.put(stringBuilder.toString(), value);
    }

    private static Position convertPosition(long packedPosition) {
        return new Position((int)(packedPosition << 0 >> 38), (int)(packedPosition << 52 >> 52), (int)(packedPosition << 26 >> 38));
    }

    private static void packExtra(ReaderBase readerBase, Consumer<MessagePackWriter> consumer) {
        try (MessageBufferPacker messageBufferPacker = MessagePack.newDefaultBufferPacker();){
            MessagePackWriter messagePackWriter = new MessagePackWriter((MessagePacker)messageBufferPacker);
            consumer.accept(messagePackWriter);
            messagePackWriter.serialize();
            try (MessageUnpacker messageUnpacker = MessagePack.newDefaultUnpacker((byte[])messageBufferPacker.toByteArray());){
                readerBase.merge(new MessagePackReader(messageUnpacker));
            }
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
    }

    private static boolean isPowerOfTwo(int value) {
        return value != 0 && (value & value - 1) == 0;
    }

    private static int smallestEncompassingPowerOfTwo(int value) {
        int i = value - 1;
        i |= i >> 1;
        i |= i >> 2;
        i |= i >> 4;
        i |= i >> 8;
        i |= i >> 16;
        return i + 1;
    }

    private static int ceilLog2(int value) {
        value = DataFixer.isPowerOfTwo(value) ? value : DataFixer.smallestEncompassingPowerOfTwo(value);
        int[] MULTIPLY_DE_BRUIJN_BIT_POSITION = new int[]{0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17, 4, 8, 31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9};
        return MULTIPLY_DE_BRUIJN_BIT_POSITION[(int)((long)value * 125613361L >> 27) & 0x1F];
    }

    private static int floorLog2(int value) {
        return DataFixer.ceilLog2(value) - (DataFixer.isPowerOfTwo(value) ? 0 : 1);
    }

    private static int unpackLongX(long packedPos) {
        return (int)(packedPos << 64 - BIT_SHIFT_X - SIZE_BITS_X >> 64 - SIZE_BITS_X);
    }

    private static int unpackLongY(long packedPos) {
        return (int)(packedPos << 64 - SIZE_BITS_Y >> 64 - SIZE_BITS_Y);
    }

    private static int unpackLongZ(long packedPos) {
        return (int)(packedPos << 64 - BIT_SHIFT_Z - SIZE_BITS_Z >> 64 - SIZE_BITS_Z);
    }

    public static Position fromLong(long packedPos) {
        return new Position(DataFixer.unpackLongX(packedPos), DataFixer.unpackLongY(packedPos), DataFixer.unpackLongZ(packedPos));
    }

    public static long asLong(Position position) {
        return DataFixer.asLong((int)position.getX(), (int)position.getY(), (int)position.getZ());
    }

    private static long asLong(int x, int y, int z) {
        long l = 0L;
        l |= ((long)x & BITS_X) << BIT_SHIFT_X;
        l |= (long)y & BITS_Y;
        return l |= ((long)z & BITS_Z) << BIT_SHIFT_Z;
    }

    static {
        SIZE_BITS_Z = SIZE_BITS_X = 1 + DataFixer.floorLog2(DataFixer.smallestEncompassingPowerOfTwo(30000000));
        BIT_SHIFT_Z = SIZE_BITS_Y = 64 - SIZE_BITS_X - SIZE_BITS_Z;
        BIT_SHIFT_X = SIZE_BITS_Y + SIZE_BITS_Z;
        BITS_X = (1L << SIZE_BITS_X) - 1L;
        BITS_Y = (1L << SIZE_BITS_Y) - 1L;
        BITS_Z = (1L << SIZE_BITS_Z) - 1L;
    }

    private static enum TrainType {
        SP1900("train_24_2"),
        SP1900_SMALL("train_20_2"),
        SP1900_MINI("train_12_2"),
        C1141A("train_24_2"),
        C1141A_SMALL("train_20_2"),
        C1141A_MINI("train_12_2"),
        M_TRAIN("train_24_2"),
        M_TRAIN_SMALL("train_19_2"),
        M_TRAIN_MINI("train_9_2"),
        CM_STOCK("train_24_2"),
        CM_STOCK_SMALL("train_19_2"),
        CM_STOCK_MINI("train_9_2"),
        MLR("train_24_2"),
        MLR_SMALL("train_20_2"),
        MLR_MINI("train_12_2"),
        MLR_CHRISTMAS("train_24_2"),
        MLR_CHRISTMAS_SMALL("train_20_2"),
        MLR_CHRISTMAS_MINI("train_12_2"),
        E44("train_24_2"),
        E44_MINI("train_12_2"),
        R_TRAIN("train_24_2"),
        R_TRAIN_SMALL("train_19_2"),
        R_TRAIN_MINI("train_9_2"),
        DRL("train_24_2"),
        K_TRAIN("train_24_2"),
        K_TRAIN_SMALL("train_19_2"),
        K_TRAIN_MINI("train_9_2"),
        K_TRAIN_TCL("train_24_2"),
        K_TRAIN_TCL_SMALL("train_19_2"),
        K_TRAIN_TCL_MINI("train_9_2"),
        K_TRAIN_AEL("train_24_2"),
        K_TRAIN_AEL_SMALL("train_19_2"),
        K_TRAIN_AEL_MINI("train_9_2"),
        C_TRAIN("train_24_2"),
        C_TRAIN_SMALL("train_19_2"),
        C_TRAIN_MINI("train_9_2"),
        S_TRAIN("train_24_2"),
        S_TRAIN_SMALL("train_19_2"),
        S_TRAIN_MINI("train_9_2"),
        A_TRAIN_TCL("train_24_2"),
        A_TRAIN_TCL_SMALL("train_19_2"),
        A_TRAIN_TCL_MINI("train_9_2"),
        A_TRAIN_AEL("train_24_2"),
        A_TRAIN_AEL_MINI("train_14_2"),
        LIGHT_RAIL_1("train_22_2"),
        LIGHT_RAIL_1_RHT("train_22_2"),
        LIGHT_RAIL_1R("train_22_2"),
        LIGHT_RAIL_1R_RHT("train_22_2"),
        LIGHT_RAIL_2("train_22_2"),
        LIGHT_RAIL_2R("train_22_2"),
        LIGHT_RAIL_2_RHT("train_22_2"),
        LIGHT_RAIL_2R_RHT("train_22_2"),
        LIGHT_RAIL_3("train_22_2"),
        LIGHT_RAIL_3_RHT("train_22_2"),
        LIGHT_RAIL_3R("train_22_2"),
        LIGHT_RAIL_3R_RHT("train_22_2"),
        LIGHT_RAIL_4("train_22_2"),
        LIGHT_RAIL_4_RHT("train_22_2"),
        LIGHT_RAIL_5("train_22_2"),
        LIGHT_RAIL_5_RHT("train_22_2"),
        LIGHT_RAIL_1R_OLD("train_22_2"),
        LIGHT_RAIL_1R_OLD_RHT("train_22_2"),
        LIGHT_RAIL_4_OLD("train_22_2"),
        LIGHT_RAIL_4_OLD_RHT("train_22_2"),
        LIGHT_RAIL_5_OLD("train_22_2"),
        LIGHT_RAIL_5_OLD_RHT("train_22_2"),
        LIGHT_RAIL_1_ORANGE("train_22_2"),
        LIGHT_RAIL_1_ORANGE_RHT("train_22_2"),
        LIGHT_RAIL_1R_ORANGE("train_22_2"),
        LIGHT_RAIL_1R_ORANGE_RHT("train_22_2"),
        LIGHT_RAIL_2_ORANGE("train_22_2"),
        LIGHT_RAIL_2_ORANGE_RHT("train_22_2"),
        LIGHT_RAIL_3_ORANGE("train_22_2"),
        LIGHT_RAIL_3_ORANGE_RHT("train_22_2"),
        LIGHT_RAIL_4_ORANGE("train_22_2"),
        LIGHT_RAIL_4_ORANGE_RHT("train_22_2"),
        LIGHT_RAIL_5_ORANGE("train_22_2"),
        LIGHT_RAIL_5_ORANGE_RHT("train_22_2"),
        LONDON_UNDERGROUND_D78("train_18_2"),
        LONDON_UNDERGROUND_D78_MINI("train_10_2"),
        LONDON_UNDERGROUND_1995("train_19_2"),
        LONDON_UNDERGROUND_1996("train_19_2"),
        R179("train_19_2"),
        R179_MINI("train_9_2"),
        R211("train_19_2"),
        R211_MINI("train_9_2"),
        R211T("train_19_2"),
        R211T_MINI("train_9_2"),
        CLASS_377_SOUTHERN("train_16_2"),
        CLASS_802_GWR("train_24_2"),
        CLASS_802_GWR_MINI("train_18_2"),
        CLASS_802_TPE("train_24_2"),
        CLASS_802_TPE_MINI("train_18_2"),
        MPL_85("train_21_2"),
        BR_423("train_15_2"),
        MINECART("train_1_1"),
        OAK_BOAT("boat_1_1"),
        SPRUCE_BOAT("boat_1_1"),
        BIRCH_BOAT("boat_1_1"),
        JUNGLE_BOAT("boat_1_1"),
        ACACIA_BOAT("boat_1_1"),
        DARK_OAK_BOAT("boat_1_1"),
        NGONG_PING_360_CRYSTAL("cable_car_1_1"),
        NGONG_PING_360_CRYSTAL_RHT("cable_car_1_1"),
        NGONG_PING_360_CRYSTAL_PLUS("cable_car_1_1"),
        NGONG_PING_360_CRYSTAL_PLUS_RHT("cable_car_1_1"),
        NGONG_PING_360_NORMAL_RED("cable_car_1_1"),
        NGONG_PING_360_NORMAL_RED_RHT("cable_car_1_1"),
        NGONG_PING_360_NORMAL_ORANGE("cable_car_1_1"),
        NGONG_PING_360_NORMAL_ORANGE_RHT("cable_car_1_1"),
        NGONG_PING_360_NORMAL_LIGHT_BLUE("cable_car_1_1"),
        NGONG_PING_360_NORMAL_LIGHT_BLUE_RHT("cable_car_1_1"),
        A320("airplane_30_3"),
        FLYING_MINECART("airplane_1_1");

        private final String baseTrainType;

        private TrainType(String baseTrainType) {
            this.baseTrainType = baseTrainType;
        }
    }

    public static enum RailType {
        WOODEN(20, false, true, true, Rail.Shape.QUADRATIC),
        STONE(40, false, true, true, Rail.Shape.QUADRATIC),
        EMERALD(60, false, true, true, Rail.Shape.QUADRATIC),
        IRON(80, false, true, true, Rail.Shape.QUADRATIC),
        OBSIDIAN(120, false, true, true, Rail.Shape.QUADRATIC),
        BLAZE(160, false, true, true, Rail.Shape.QUADRATIC),
        QUARTZ(200, false, true, true, Rail.Shape.QUADRATIC),
        DIAMOND(300, false, true, true, Rail.Shape.QUADRATIC),
        PLATFORM(80, true, false, true, Rail.Shape.QUADRATIC),
        SIDING(40, true, false, true, Rail.Shape.QUADRATIC),
        TURN_BACK(80, false, false, true, Rail.Shape.QUADRATIC),
        CABLE_CAR(30, false, true, true, Rail.Shape.CABLE),
        CABLE_CAR_STATION(2, false, true, true, Rail.Shape.QUADRATIC),
        RUNWAY(300, false, true, false, Rail.Shape.QUADRATIC),
        AIRPLANE_DUMMY(300, false, true, false, Rail.Shape.QUADRATIC),
        NONE(0, false, false, true, Rail.Shape.QUADRATIC);

        public final int speedLimitKilometersPerHour;
        public final double speedLimitMetersPerMillisecond;
        public final boolean hasSavedRail;
        public final boolean canAccelerate;
        public final boolean canHaveSignal;
        public final Rail.Shape shape;

        private RailType(int speedLimitKilometersPerHour, boolean hasSavedRail, boolean canAccelerate, boolean canHaveSignal, Rail.Shape shape) {
            this.speedLimitKilometersPerHour = speedLimitKilometersPerHour;
            this.speedLimitMetersPerMillisecond = Utilities.kilometersPerHourToMetersPerMillisecond(speedLimitKilometersPerHour);
            this.hasSavedRail = hasSavedRail;
            this.canAccelerate = canAccelerate;
            this.canHaveSignal = canHaveSignal;
            this.shape = shape;
        }
    }
}

