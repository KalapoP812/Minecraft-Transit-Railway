package org.mtr.mod.packet;

import org.mtr.mapping.holder.BlockPos;
import org.mtr.mapping.holder.MinecraftServer;
import org.mtr.mapping.holder.Property;
import org.mtr.mapping.holder.ServerPlayerEntity;
import org.mtr.mapping.registry.PacketHandler;
import org.mtr.mapping.tool.PacketBufferReceiver;
import org.mtr.mapping.tool.PacketBufferSender;
import org.mtr.mod.Init;
import org.mtr.mod.block.BlockNode;

public final class PacketUpdateNodeCustomAngle extends PacketHandler {

	private final BlockPos blockPos;
	private final float customAngle;

	public PacketUpdateNodeCustomAngle(PacketBufferReceiver packetBufferReceiver) {
		blockPos = BlockPos.fromLong(packetBufferReceiver.readLong());
		customAngle = packetBufferReceiver.readFloat();
	}

	public PacketUpdateNodeCustomAngle(BlockPos blockPos, float customAngle) {
		this.blockPos = blockPos;
		this.customAngle = customAngle;
	}

	@Override
	public void write(PacketBufferSender packetBufferSender) {
		packetBufferSender.writeLong(blockPos.asLong());
		packetBufferSender.writeFloat(customAngle);
	}

	@Override
	public void runServer(MinecraftServer minecraftServer, ServerPlayerEntity serverPlayerEntity) {
		if (!Init.isChunkLoaded(serverPlayerEntity.getEntityWorld(), blockPos)) {
			return;
		}

		final org.mtr.mapping.holder.BlockState blockState = serverPlayerEntity.getEntityWorld().getBlockState(blockPos);
		if (!(blockState.getBlock().data instanceof BlockNode) || blockState.get(new Property<>(BlockNode.IS_CONNECTED.data))) {
			return;
		}

		final org.mtr.mapping.holder.BlockEntity blockEntity = serverPlayerEntity.getEntityWorld().getBlockEntity(blockPos);
		if (blockEntity != null && blockEntity.data instanceof BlockNode.BlockEntity) {
			((BlockNode.BlockEntity) blockEntity.data).setCustomAngle(customAngle);
		}
	}
}
