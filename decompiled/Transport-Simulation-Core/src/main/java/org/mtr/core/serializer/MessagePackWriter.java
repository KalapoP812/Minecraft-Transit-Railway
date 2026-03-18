/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.org.msgpack.core.MessagePacker
 */
package org.mtr.core.serializer;

import javax.annotation.Nullable;
import org.mtr.core.Main;
import org.mtr.core.serializer.WriterBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.org.msgpack.core.MessagePacker;

public final class MessagePackWriter
extends WriterBase {
    private final MessagePacker messagePacker;
    private final ObjectArrayList<Pack> instructions = new ObjectArrayList();

    public MessagePackWriter(MessagePacker messagePacker) {
        this.messagePacker = messagePacker;
    }

    @Override
    public void writeBoolean(String key, boolean value) {
        this.pack(key, () -> this.messagePacker.packBoolean(value));
    }

    @Override
    public void writeInt(String key, int value) {
        this.pack(key, () -> this.messagePacker.packInt(value));
    }

    @Override
    public void writeLong(String key, long value) {
        this.pack(key, () -> this.messagePacker.packLong(value));
    }

    @Override
    public void writeDouble(String key, double value) {
        this.pack(key, () -> this.messagePacker.packDouble(value));
    }

    @Override
    public void writeString(String key, String value) {
        this.pack(key, () -> this.messagePacker.packString(value));
    }

    @Override
    public WriterBase.Array writeArray(String key) {
        MessagePackArrayWriter messagePackerArrayWriter = new MessagePackArrayWriter(this.messagePacker);
        this.pack(key, () -> this.messagePacker.packArrayHeader(messagePackerArrayWriter.instructions.size()), () -> messagePackerArrayWriter.serializePart());
        return messagePackerArrayWriter;
    }

    @Override
    public WriterBase writeChild(String key) {
        MessagePackWriter messagePackerWriter = new MessagePackWriter(this.messagePacker);
        this.pack(key, () -> this.messagePacker.packMapHeader(messagePackerWriter.instructions.size()), messagePackerWriter::serializePart);
        return messagePackerWriter;
    }

    public void serialize() {
        try {
            this.messagePacker.packMapHeader(this.instructions.size());
            this.serializePart();
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
    }

    private void pack(String key, Pack instruction) {
        this.pack(key, instruction, null);
    }

    private void pack(String key, Pack instruction, @Nullable Pack additionalInstructions) {
        this.instructions.add(() -> {
            this.messagePacker.packString(key);
            instruction.pack();
            if (additionalInstructions != null) {
                additionalInstructions.pack();
            }
        });
    }

    private void serializePart() throws Exception {
        for (Pack instruction : this.instructions) {
            instruction.pack();
        }
    }

    @FunctionalInterface
    private static interface Pack {
        public void pack() throws Exception;
    }

    private static final class MessagePackArrayWriter
    extends WriterBase.Array {
        private final MessagePacker messagePacker;
        private final ObjectArrayList<Pack> instructions = new ObjectArrayList();

        private MessagePackArrayWriter(MessagePacker messagePacker) {
            this.messagePacker = messagePacker;
        }

        @Override
        public void writeBoolean(boolean value) {
            this.pack(() -> this.messagePacker.packBoolean(value));
        }

        @Override
        public void writeInt(int value) {
            this.pack(() -> this.messagePacker.packInt(value));
        }

        @Override
        public void writeLong(long value) {
            this.pack(() -> this.messagePacker.packLong(value));
        }

        @Override
        public void writeDouble(double value) {
            this.pack(() -> this.messagePacker.packDouble(value));
        }

        @Override
        public void writeString(String value) {
            this.pack(() -> this.messagePacker.packString(value));
        }

        @Override
        public WriterBase writeChild() {
            MessagePackWriter messagePackerWriter = new MessagePackWriter(this.messagePacker);
            this.pack(() -> this.messagePacker.packMapHeader(messagePackerWriter.instructions.size()), () -> messagePackerWriter.serializePart());
            return messagePackerWriter;
        }

        private void pack(Pack instruction) {
            this.pack(instruction, null);
        }

        private void pack(Pack instruction, @Nullable Pack additionalInstructions) {
            this.instructions.add(() -> {
                instruction.pack();
                if (additionalInstructions != null) {
                    additionalInstructions.pack();
                }
            });
        }

        private void serializePart() throws Exception {
            for (Pack instruction : this.instructions) {
                instruction.pack();
            }
        }
    }
}

