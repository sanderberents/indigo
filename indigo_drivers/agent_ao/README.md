# Adaptive optics agent

Backend implementation of adaptive optics (AO) control. The agent drives an AO with the images of an off-axis guide camera (OAG) behind it, and slowly hands the deflection of the AO over to the mount.

## Supported devices

- Any INDIGO AO device (interface `AO`): Starlight Xpress AO (`indigo_ao_sx`, relative steps), SBIG AO-7 (`indigo_ccd_sbig`, absolute percent) and the AO of the camera simulator.
- Any INDIGO camera as the OAG. Cameras that support `CCD_STREAMING` are tracked at their own frame rate, other cameras with single exposures, which is usually slower.
- Optionally an INDIGO guide port (for the direct mount bump) and an INDIGO mount (declination, side of pier and slews are read, the mount is never written).

## Supported platforms

This driver is platform independent.

## License

INDIGO Astronomy open-source license.

## Use

indigo_server indigo_agent_ao indigo_ao_sx indigo_ccd_... indigo_mount_...

The OAG must be behind the AO, so that it sees the deflection of the AO (closed loop). The agent can be used in two ways, which differ in who moves the mount.

### Independent, as a guider

The agent is the only guider. The OAG behind the AO is tracked at the frame rate of the camera, the AO counteracts the fast star motion, and the agent bumps the mount with slow guide pulses of its own when the AO drifts away from its centre (differential flexure, periodic error, polar misalignment). The Guider Agent is not needed and must not be guiding. Select the OAG in `FILTER_CCD_LIST`, the AO in `FILTER_AO_LIST`, the guide port of the mount in `FILTER_GUIDER_LIST` and, for the declination, the side of the pier and the slews, the mount in `FILTER_MOUNT_LIST`. Run `CALIBRATION` and `BUMP_CALIBRATION`, then `TRACKING` with `AGENT_AO_BUMP_MODE` set to `DIRECT`. With `OFF` the mount is never bumped: the AO follows the star until it reaches the maximum deflection, where it stays and `AGENT_AO_STATS.SATURATED` is set. Stopping and starting the loop centers the AO again.

To dither between exposures, relate the "Imager Agent" in `FILTER_RELATED_AGENT_LIST` and set `DITHER_AMPLITUDE` (in pixels of the OAG, 0 disables dithering) and `DITHER_FRAMES` (exposures between dithers). When an exposure of the Imager Agent finishes, the AO tilts the star by a random offset within that amplitude, which needs no mount motion, and the loop keeps the star there; the mount bump ignores the offset. The Imager Agent itself does not dither without a Guider Agent and does not wait for the AO, so the tilt has to settle while the image is downloaded. Set the delay after each exposure of the Imager Agent (`AGENT_IMAGER_BATCH.DELAY`, 1 second is usually enough) to give the AO time to settle before the next exposure, and keep the amplitude small. The current offset is shown in `AGENT_AO_STATS.DITHER_X` and `DITHER_Y`. The dither range is part of `RANGE`.

### Together with the Guider Agent

The Guider Agent keeps its own guidescope with its own guide camera, and is the only one that guides and moves the mount, as usual. The AO Agent only controls the AO, using the OAG behind it, and never pulses the mount while the Guider Agent is busy. Relate the "Guider Agent" in `FILTER_RELATED_AGENT_LIST` and select the OAG and the AO as above (the guide port and the mount are not needed for the AO itself). Run `CALIBRATION`, calibrate and start the Guider Agent as usual, then run `TRACKING`. The AO only has to absorb what the guidescope cannot see, such as differential flexure and seeing, which stays well within its range; stopping and starting the loop centers the AO again between exposures. The AO loop pauses while the Guider Agent dithers and looks for its star again afterwards. Dithering is left to the Guider Agent: the AO Agent does not dither at all while a Guider Agent is related.

### Processes

Set `EXPOSURE` in `AGENT_AO_SETTINGS` and a small `CCD_FRAME` of the camera around a guide star. The processes are started with `AGENT_START_PROCESS` and stopped with `AGENT_ABORT_PROCESS`:

1. `EXERCISE` moves the AO to the four corners of its range and back to the centre with an interval, so that the AO can be watched working. `CENTER` centers the AO.
2. `CALIBRATION` moves the AO along both axes while the star is measured, and stores the matrix of the AO (`RA_X`, `RA_Y`, `DEC_X`, `DEC_Y` in pixels per step, with `ANGLE`, `SCALE_RA` and `SCALE_DEC` derived from it). It needs a star in the camera, no mount and no guider.
3. `BUMP_CALIBRATION` pulses the mount in all directions while the AO holds the centre, and stores the orientation of the OAG image on the sky, the guide speeds and the side of the pier (`OAG_ANGLE`, `SPEED_RA`, `SPEED_DEC`, `OAG_SIDE_OF_PIER`, `OAG_COS_DEC`). It needs a tracking mount and an idle Guider Agent.
4. `TRACKING` starts the loop: for every frame the agent measures how far the star has moved from its reference position and moves the AO to reduce that displacement by the fraction `GAIN`, ignoring displacements smaller than `MIN_MOVE`. The slow part of the AO deflection is passed to the mount. `AGENT_AO_STATS` shows the frame rate, the residual displacement and its RMSE, the AO position and deflection, and the mount bump.
5. `DARK_FRAME` takes `DARK_FRAME_COUNT` (at least 3) dark frames with the `EXPOSURE` and the `CCD_FRAME` of the camera and keeps their per pixel median. The camera is switched to frame type `DARK`, so cover the OAG unless the camera closes a shutter. While the dark frame fits the exposure and the frame (size, binning, pixel format) of the images, it is subtracted from every image before the star positions are measured, in all processes, and `AGENT_AO_STATS.DARK_ACTIVE` is set. The dark frame is optional, is kept in memory only and is not used when the exposure or the frame changes. `DARK_RESET` discards it.

`AGENT_AO_MODE` selects relative steps or absolute percent positioning; for the SBIG AO-7 it is set automatically from the driver name. `AGENT_AO_BUMP_MODE` selects whether the mount is bumped with guide pulses of this agent (`DIRECT`) or not at all (`OFF`).

## Status: Under development

## Notes

- The mount is moved by one party at a time. While the Guider Agent is previewing, calibrating or guiding, this agent sends no pulses. Otherwise the agent sends slow guide pulses itself, at most one pulse per `BUMP_INTERVAL`, limited by `MIN_PULSE` and `MAX_PULSE` and by `AGENT_AO_DEC_MODE`.
- The correction rate of the AO is the frame rate of the camera. A mount pulse can only be sent between frames, so keep `MAX_PULSE` short.
- Conventions: all vectors are star displacements in pixels of the OAG image. The RA axis is the direction in which the star moves when the mount is pulsed west, positive Dec is the direction of a north pulse. An AO axis is positive along the `EAST` and `NORTH` items. These are assumptions to confirm with the hardware; the calibrations measure the actual directions.
- The loop pauses and centers the AO while the mount slews or the Guider Agent dithers, and looks for the star again afterwards. After `LOST_FRAMES` frames without a star the AO is centered and a new star is selected.
- The calibration pulses move the star several pixels between frames. If the star is lost during `BUMP_CALIBRATION`, increase `SELECTION_RADIUS` or shorten `BUMP_CALIBRATION_PULSE`.
- The deflection of the AO is limited by `RANGE`, the maximum deflection in AO steps, or in percent for an absolute AO such as the SBIG AO-7. The distance of the AO position from the centre never exceeds it, in the loop, the exercise (whose amplitude is a percentage of it) and the calibration, to keep the tilt of the optics under control. When the loop asks for more, the AO stays at the limit, `AGENT_AO_STATS.SATURATED` shows it and the mount bump takes over the long-term part.
- Settings, including the calibration results, are saved with the agent configuration.
