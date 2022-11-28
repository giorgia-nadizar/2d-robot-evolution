package it.units.erallab.robotevo2d.main;

import it.units.erallab.mrsim2d.viewer.Drawer;
import it.units.erallab.mrsim2d.viewer.VideoUtils;
import it.units.malelab.jnb.core.Param;

import java.util.function.Function;

/**
 * @author "Eric Medvet" on 2022/09/01 for 2d-robot-evolution
 */
public record VideoSaver(
    @Param(value = "w", dI = 400) int w,
    @Param(value = "h", dI = 250) int h,
    @Param(value = "frameRate", dD = 30) double frameRate,
    @Param(value = "startTime", dD = 0) double startTime,
    @Param(value = "endTime", dD = 30) double endTime,
    @Param(value = "codec", dS = "jcodec") VideoUtils.EncoderFacility codec,
    @Param(value = "drawer") Function<String, Drawer> drawer
) {}