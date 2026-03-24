package org.mtr.mod.screen;

import org.mtr.mapping.holder.*;
import org.mtr.mapping.mapper.ButtonWidgetExtension;
import org.mtr.mapping.mapper.GraphicsHolder;
import org.mtr.mapping.mapper.TextHelper;
import org.mtr.mapping.mapper.TextFieldWidgetExtension;
import org.mtr.mapping.tool.TextCase;
import org.mtr.mod.InitClient;
import org.mtr.mod.block.BlockNode;
import org.mtr.mod.client.IDrawing;
import org.mtr.mod.data.IGui;
import org.mtr.mod.packet.PacketUpdateNodeCustomAngle;

import java.util.Locale;

public class NodeCustomAngleScreen extends MTRScreenBase implements IGui {

	private final BlockPos blockPos;
	private final TextFieldWidgetExtension textFieldAngle;
	private final ButtonWidgetExtension buttonDone;

	private static final String ANGLE_REGEX = "[^0-9+\\-\\.]";

	public NodeCustomAngleScreen(BlockPos blockPos) {
		super();
		this.blockPos = blockPos;

		textFieldAngle = new TextFieldWidgetExtension(0, 0, 0, SQUARE_SIZE, 32, TextCase.DEFAULT, ANGLE_REGEX, "0");
		buttonDone = new ButtonWidgetExtension(0, 0, 0, SQUARE_SIZE, TextHelper.translatable("gui.done"), button -> submit());
	}

	@Override
	protected void init2() {
		super.init2();

		final int panelWidth = SQUARE_SIZE * 10;
		final int x = (width - panelWidth) / 2;
		final int y = height / 2 - SQUARE_SIZE;

		IDrawing.setPositionAndWidth(textFieldAngle, x, y, panelWidth);
		IDrawing.setPositionAndWidth(buttonDone, x, y + SQUARE_SIZE + TEXT_FIELD_PADDING, panelWidth);

		final ClientWorld clientWorld = MinecraftClient.getInstance().getWorldMapped();
		if (clientWorld != null) {
			textFieldAngle.setText2(formatAngle(BlockNode.getAngle(clientWorld, blockPos, clientWorld.getBlockState(blockPos))));
		}
		textFieldAngle.setChangedListener2(text -> buttonDone.setActiveMapped(parseAngle(text) != null));
		buttonDone.setActiveMapped(parseAngle(textFieldAngle.getText2()) != null);

		addChild(new ClickableWidget(textFieldAngle));
		addChild(new ClickableWidget(buttonDone));
	}

	@Override
	public void tick2() {
		super.tick2();
		textFieldAngle.tick2();
	}

	@Override
	public void render(GraphicsHolder graphicsHolder, int mouseX, int mouseY, float delta) {
		renderBackground(graphicsHolder);
		super.render(graphicsHolder, mouseX, mouseY, delta);
		graphicsHolder.drawText(TextHelper.literal("Custom Angle"), textFieldAngle.getX2(), textFieldAngle.getY2() - 10 - TEXT_PADDING, ARGB_WHITE, false, GraphicsHolder.getDefaultLight());
	}

	@Override
	public boolean isPauseScreen2() {
		return false;
	}

	private void submit() {
		final Float parsedAngle = parseAngle(textFieldAngle.getText2());
		if (parsedAngle != null) {
			final float roundedAngle = Math.round(parsedAngle * 1000F) / 1000F;
			InitClient.REGISTRY_CLIENT.sendPacketToServer(new PacketUpdateNodeCustomAngle(blockPos, roundedAngle));
			onClose2();
		}
	}

	private static String formatAngle(float angle) {
		return String.format(Locale.ROOT, "%.3f", angle);
	}

	private static Float parseAngle(String text) {
		try {
			return Float.parseFloat(text);
		} catch (Exception ignored) {
			return null;
		}
	}
}
