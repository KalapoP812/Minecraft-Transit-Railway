/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntIntImmutablePair
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2IntAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectArrayMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet
 *  org.mtr.libraries.org.msgpack.core.MessageBufferPacker
 *  org.mtr.libraries.org.msgpack.core.MessagePack
 *  org.mtr.libraries.org.msgpack.core.MessagePacker
 *  org.mtr.libraries.org.msgpack.core.MessageTypeException
 *  org.mtr.libraries.org.msgpack.core.MessageUnpacker
 */
package org.mtr.core.simulation;

import java.io.InputStream;
import java.io.OutputStream;
import java.nio.file.DirectoryNotEmptyException;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.OpenOption;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.nio.file.attribute.FileAttribute;
import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Function;
import java.util.stream.Stream;
import javax.annotation.Nullable;
import org.mtr.core.Main;
import org.mtr.core.serializer.MessagePackReader;
import org.mtr.core.serializer.MessagePackWriter;
import org.mtr.core.serializer.SerializedDataBaseWithId;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntIntImmutablePair;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2IntAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectArrayMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet;
import org.mtr.libraries.org.msgpack.core.MessageBufferPacker;
import org.mtr.libraries.org.msgpack.core.MessagePack;
import org.mtr.libraries.org.msgpack.core.MessagePacker;
import org.mtr.libraries.org.msgpack.core.MessageTypeException;
import org.mtr.libraries.org.msgpack.core.MessageUnpacker;

public class FileLoader<T extends SerializedDataBaseWithId> {
    public final String key;
    private final ObjectSet<T> dataSet;
    private final Path path;
    private final boolean threadedFileLoading;
    private final Object2IntAVLTreeMap<String> fileHashes = new Object2IntAVLTreeMap();

    public FileLoader(ObjectSet<T> dataSet, Function<MessagePackReader, T> getData, Path rootPath, String key, boolean threadedFileLoading) {
        this.key = key;
        this.dataSet = dataSet;
        this.path = rootPath.resolve(key);
        FileLoader.createDirectory(this.path);
        this.threadedFileLoading = threadedFileLoading;
        this.readMessagePackFromFile(getData);
    }

    @Deprecated
    public FileLoader(ObjectSet<T> dataSet, Function<MessagePackReader, T> getData, Path rootPath, String key) {
        this(dataSet, getData, rootPath, key, false);
    }

    public IntIntImmutablePair save(boolean useReducedHash) {
        ObjectArrayList<T> dirtyData = new ObjectArrayList<>(this.dataSet);
        ObjectImmutableList<ObjectArrayList<String>> checkFilesToDelete = FileLoader.createEmptyList256();
        this.fileHashes.keySet().forEach(fileName -> ((ObjectArrayList)checkFilesToDelete.get(FileLoader.getParentInt(fileName))).add(fileName));
        int filesWritten = this.writeDirtyDataToFile(checkFilesToDelete, dirtyData, SerializedDataBaseWithId::getHexId, useReducedHash);
        int filesDeleted = 0;
        for (ObjectArrayList<String> checkFilesToDeleteForParent : checkFilesToDelete) {
            for (String fileName2 : checkFilesToDeleteForParent) {
                try {
                    if (Files.deleteIfExists(this.path.resolve(fileName2))) {
                        ++filesDeleted;
                    }
                }
                catch (Exception e) {
                    Main.LOGGER.error("", (Throwable)e);
                }
                this.fileHashes.removeInt(fileName2);
            }
        }
        return new IntIntImmutablePair(filesWritten, filesDeleted);
    }

    private void readMessagePackFromFile(Function<MessagePackReader, T> getData) {
        Object2ObjectArrayMap<String, Future<T>> futureDataMap = new Object2ObjectArrayMap<>();
        ExecutorService executorService = Executors.newCachedThreadPool();
        try (Stream<Path> pathStream = Files.list(this.path);){
            pathStream.forEach(idFolder -> {
                try (Stream<Path> folderStream = Files.list(idFolder);){
                    folderStream.forEach(idFile -> {
                        String fileName = FileLoader.combineAsPath(idFolder, idFile);
                        if (this.threadedFileLoading) {
                            futureDataMap.put(fileName, executorService.submit(() -> FileLoader.readFile(getData, idFile)));
                        } else {
                            this.processFile(fileName, FileLoader.readFile(getData, idFile));
                        }
                    });
                }
                catch (Exception e) {
                    Main.LOGGER.error("", (Throwable)e);
                }
                try {
                    Files.deleteIfExists(idFolder);
                    Main.LOGGER.debug("Deleted empty folder: {}", idFolder);
                }
                catch (DirectoryNotEmptyException e) {
                }
                catch (Exception e) {
                    Main.LOGGER.error("", (Throwable)e);
                }
            });
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
        futureDataMap.forEach((fileName, futureData) -> {
            try {
                this.processFile(fileName, futureData.get());
            }
            catch (Exception e) {
                Main.LOGGER.error("", (Throwable)e);
            }
        });
    }

    private void processFile(String fileName, @Nullable T data) {
        try {
            if (data != null) {
                if (data.isValid()) {
                    this.dataSet.add(data);
                } else {
                    Main.LOGGER.warn("Skipping invalid data: {}", data);
                }
                this.fileHashes.put(fileName, FileLoader.getHash(data, true));
            }
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
    }

    private int writeDirtyDataToFile(ObjectImmutableList<ObjectArrayList<String>> checkFilesToDelete, ObjectArrayList<T> dirtyData, Function<T, String> getFileName, boolean useReducedHash) {
        int filesWritten = 0;
        while (!dirtyData.isEmpty()) {
            T data = dirtyData.remove(0);
            if (data == null || !data.isValid()) continue;
            String fileName = getFileName.apply(data);
            String parentFolderName = FileLoader.getParent(fileName);
            String parentAndFileName = FileLoader.combineAsPath(parentFolderName, fileName);
            int hash = FileLoader.getHash(data, useReducedHash);
            if (!this.fileHashes.containsKey(parentAndFileName) || hash != this.fileHashes.getInt(parentAndFileName)) {
                FileLoader.createDirectory(this.path.resolve(parentFolderName));
                try (MessagePacker messagePacker = MessagePack.newDefaultPacker((OutputStream)Files.newOutputStream(this.path.resolve(parentAndFileName), StandardOpenOption.CREATE));){
                    FileLoader.packMessage(messagePacker, data, useReducedHash);
                }
                catch (Exception e) {
                    Main.LOGGER.error("", (Throwable)e);
                }
                this.fileHashes.put(parentAndFileName, hash);
                ++filesWritten;
            }
            ((ObjectArrayList)checkFilesToDelete.get(FileLoader.getParentInt(fileName))).remove(parentAndFileName);
        }
        return filesWritten;
    }

    @Nullable
    private static <T extends SerializedDataBaseWithId> T readFile(Function<MessagePackReader, T> getData, Path idFile) {
        try (InputStream inputStream = Files.newInputStream(idFile, new OpenOption[0]);
             MessageUnpacker messageUnpacker = MessagePack.newDefaultUnpacker((InputStream)inputStream)) {
            return getData.apply(new MessagePackReader(messageUnpacker));
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
            if (e instanceof MessageTypeException) {
                Main.LOGGER.info("Deleting file with parsing error [{}]", idFile);
                try {
                    Files.deleteIfExists(idFile);
                }
                catch (Exception deleteException) {
                    Main.LOGGER.error("", (Throwable)deleteException);
                }
            }
            return null;
        }
    }

    private static String getParent(String fileName) {
        return fileName.substring(Math.max(0, fileName.length() - 2));
    }

    private static int getParentInt(String fileName) {
        try {
            return Integer.parseInt(FileLoader.getParent(fileName), 16);
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
            return 0;
        }
    }

    private static ObjectImmutableList<ObjectArrayList<String>> createEmptyList256() {
        ObjectArrayList list = new ObjectArrayList();
        for (int i = 0; i <= 255; ++i) {
            list.add(new ObjectArrayList());
        }
        return new ObjectImmutableList((ObjectList)list);
    }

    private static String combineAsPath(Path parentFolderPath, Path filePath) {
        return FileLoader.combineAsPath(parentFolderPath.getFileName().toString(), filePath.getFileName().toString());
    }

    private static String combineAsPath(String parentFolderName, String fileName) {
        return String.format("%s/%s", parentFolderName, fileName);
    }

    private static int getHash(SerializedDataBaseWithId data, boolean useReducedHash) {
        int hash = 0;
        try (MessageBufferPacker messageBufferPacker = MessagePack.newDefaultBufferPacker();){
            FileLoader.packMessage((MessagePacker)messageBufferPacker, data, useReducedHash);
            hash = Arrays.hashCode(messageBufferPacker.toByteArray());
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
        return hash;
    }

    private static void packMessage(MessagePacker messagePacker, SerializedDataBaseWithId data, boolean useReducedHash) {
        MessagePackWriter messagePackWriter = new MessagePackWriter(messagePacker);
        if (useReducedHash) {
            data.serializeData(messagePackWriter);
        } else {
            data.serializeFullData(messagePackWriter);
        }
        messagePackWriter.serialize();
    }

    private static void createDirectory(Path path) {
        if (!Files.exists(path, new LinkOption[0])) {
            try {
                Files.createDirectories(path, new FileAttribute[0]);
            }
            catch (Exception e) {
                Main.LOGGER.error("", (Throwable)e);
            }
        }
    }
}

