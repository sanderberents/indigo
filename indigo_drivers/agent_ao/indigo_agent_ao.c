// Copyright (c) 2026 INDIGO initiative
// All rights reserved.
//
// You may use this software under the terms of 'INDIGO Astronomy
// open-source license' (see LICENSE.md).
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHORS 'AS IS' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/** INDIGO Adaptive optics agent
 \file indigo_agent_ao.c

 The agent owns an off-axis guide camera (OAG) behind the adaptive optics. Its images are tracked at the rate of the
 camera (CCD_STREAMING, a series of single exposures as a fallback) and the AO is moved to reduce the star displacement d (gain GAIN per frame, dead-band MIN_MOVE),
 which makes a closed loop: the AO deflection a (in OAG pixels) is part of the star position the camera sees. The slow
 part of the deflection is handed over to the mount, "bumped", so that the AO relaxes to its centre: the total mount
 displacement T integrates a with a long time constant, and is realised by slow, rare pulses to a guide port selected in
 this agent. While a related Guider Agent is busy, this agent never moves the mount and pauses the loop while it dithers.

 A related Guider Agent does the dithering. Without one, this agent dithers for a related Imager Agent: when an exposure of the Imager Agent has
 finished, the AO tilts the star by a small random offset (the dither d, in OAG pixels) by moving the reference of the
 loop, which needs no mount motion. The mount bump ignores d.

 Conventions: all vectors are star displacements in the pixels of the OAG frame. The RA axis is the direction in which
 the star moves when the mount is pulsed west, positive Dec is the direction of a north pulse. An AO axis is positive
 along the EAST and NORTH item (relative mode: steps, absolute mode: percent of full deflection).
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <sys/types.h>

#include <indigo/indigo_driver_xml.h>
#include <indigo/indigo_filter.h>
#include <indigo/indigo_ccd_driver.h>
#include <indigo/indigo_raw_utils.h>

#include "indigo_agent_ao.h"

#define DRIVER_VERSION 0x03000001
#define DRIVER_NAME "indigo_agent_ao"

#define MAX_STARS 50
#define MIN_DARK_FRAMES 3
#define MAX_DARK_FRAMES 50
#define DARK_SKIP_FRAMES 1
#define PREVIEW_PERIOD 1.0
#define FRAME_TIMEOUT 3.0
#define STATS_PERIOD 0.5
#define GUIDER_AGENT_PREFIX "Guider Agent"
#define IMAGER_AGENT_PREFIX "Imager Agent"
#define DITHER_SKIP_FRAMES 2

#if defined(INDIGO_WINDOWS)
static double drand48() {
	return (double)rand() / (RAND_MAX + 1.0);
}
#endif

#define DEVICE_PRIVATE_DATA ((ao_agent_private_data *)device->private_data)
#define CLIENT_PRIVATE_DATA ((ao_agent_private_data *)FILTER_CLIENT_CONTEXT->device->private_data)

// -------------------------------------------------------------------------------- settings

typedef struct {
	const char *name;
	const char *label;
	double min, max, step, value;
} setting_definition;

enum {
	S_EXPOSURE,
	S_RADIUS,
	S_GAIN,
	S_MIN_MOVE,
	S_RANGE,
	S_MAX_STEP,
	S_CAL_STEP,
	S_CAL_SETTLE,
	S_CAL_FRAMES,
	S_EX_AMPLITUDE,
	S_EX_INTERVAL,
	S_EX_CYCLES,
	S_LOST_FRAMES,
	S_AO_TIMEOUT,
	S_BUMP_TAU,
	S_BUMP_GAIN,
	S_BUMP_INTERVAL,
	S_BUMP_THRESHOLD,
	S_MIN_PULSE,
	S_MAX_PULSE,
	S_CAL_PULSE,
	S_DITHER_AMPLITUDE,
	S_DITHER_FRAMES,
	S_DARK_FRAMES,
	S_RA_X,
	S_RA_Y,
	S_DEC_X,
	S_DEC_Y,
	S_RA_MINUS_X,
	S_RA_MINUS_Y,
	S_DEC_MINUS_X,
	S_DEC_MINUS_Y,
	S_OAG_SOP,
	S_WEST_X,
	S_WEST_Y,
	S_EAST_X,
	S_EAST_Y,
	S_NORTH_X,
	S_NORTH_Y,
	S_SOUTH_X,
	S_SOUTH_Y,
	S_COUNT
};

static const setting_definition settings_definitions[S_COUNT] = {
	{ "EXPOSURE", "Exposure (s)", 0.001, 2, 0, 0.1 },
	{ "SELECTION_RADIUS", "Selection radius (px)", 4, 64, 1, 10 },
	{ "GAIN", "Loop gain", 0.05, 1, 0, 0.5 },
	{ "MIN_MOVE", "Minimum move (px)", 0, 5, 0, 0.05 },
	{ "RANGE", "Maximum AO deflection (steps or %)", 1, 1000, 0, 50 },
	{ "MAX_STEP", "Maximum step per request", 1, 1000, 0, 50 },
	{ "CALIBRATION_STEP", "AO calibration step", 1, 200, 0, 10 },
	{ "CALIBRATION_SETTLE", "Settle time (s)", 0, 10, 0, 0.5 },
	{ "CALIBRATION_FRAMES", "Frames averaged", 1, 100, 1, 5 },
	{ "EXERCISE_AMPLITUDE", "Exercise amplitude (% of the maximum deflection)", 1, 100, 0, 50 },
	{ "EXERCISE_INTERVAL", "Exercise interval (s)", 0.1, 60, 0, 2 },
	{ "EXERCISE_CYCLES", "Exercise cycles (0 = until aborted)", 0, 1000, 1, 3 },
	{ "LOST_FRAMES", "Lost star frames limit", 1, 1000, 1, 30 },
	{ "AO_TIMEOUT", "AO timeout (s)", 0.5, 60, 0, 5 },
	{ "BUMP_TIME_CONSTANT", "Bump time constant (s)", 1, 600, 0, 30 },
	{ "BUMP_GAIN", "Bump gain", 0, 2, 0, 1 },
	{ "BUMP_INTERVAL", "Direct bump interval (s)", 0.5, 600, 0, 5 },
	{ "BUMP_THRESHOLD", "Direct bump threshold (px)", 0.05, 50, 0, 0.5 },
	{ "MIN_PULSE", "Minimum pulse (s)", 0.001, 5, 0, 0.02 },
	{ "MAX_PULSE", "Maximum pulse (s)", 0.01, 30, 0, 1 },
	{ "BUMP_CALIBRATION_PULSE", "Mount calibration pulse (s)", 0.05, 30, 0, 1 },
	{ "DITHER_AMPLITUDE", "Dither amplitude (px, 0 = no dithering)", 0, 50, 0, 0 },
	{ "DITHER_FRAMES", "Exposures between dithers", 1, 1000, 1, 1 },
	{ "DARK_FRAME_COUNT", "Dark frames to combine", MIN_DARK_FRAMES, MAX_DARK_FRAMES, 1, 5 },
	{ "RA_X", "AO RA axis X (px/step)", -1000, 1000, 0, 0 },
	{ "RA_Y", "AO RA axis Y (px/step)", -1000, 1000, 0, 0 },
	{ "DEC_X", "AO Dec axis X (px/step)", -1000, 1000, 0, 0 },
	{ "DEC_Y", "AO Dec axis Y (px/step)", -1000, 1000, 0, 0 },
	{ "RA_MINUS_X", "AO -RA direction X (px/step)", -1000, 1000, 0, 0 },
	{ "RA_MINUS_Y", "AO -RA direction Y (px/step)", -1000, 1000, 0, 0 },
	{ "DEC_MINUS_X", "AO -Dec direction X (px/step)", -1000, 1000, 0, 0 },
	{ "DEC_MINUS_Y", "AO -Dec direction Y (px/step)", -1000, 1000, 0, 0 },
	{ "OAG_SIDE_OF_PIER", "Side of pier at calibration", -1, 1, 1, 0 },
	{ "WEST_X", "OAG +RA (west) pulse X (px/s at the equator)", -1000, 1000, 0, 0 },
	{ "WEST_Y", "OAG +RA (west) pulse Y (px/s at the equator)", -1000, 1000, 0, 0 },
	{ "EAST_X", "OAG -RA (east) pulse X (px/s at the equator)", -1000, 1000, 0, 0 },
	{ "EAST_Y", "OAG -RA (east) pulse Y (px/s at the equator)", -1000, 1000, 0, 0 },
	{ "NORTH_X", "OAG +Dec (north) pulse X (px/s)", -1000, 1000, 0, 0 },
	{ "NORTH_Y", "OAG +Dec (north) pulse Y (px/s)", -1000, 1000, 0, 0 },
	{ "SOUTH_X", "OAG -Dec (south) pulse X (px/s)", -1000, 1000, 0, 0 },
	{ "SOUTH_Y", "OAG -Dec (south) pulse Y (px/s)", -1000, 1000, 0, 0 }
};

enum {
	T_PHASE,
	T_FRAME_RATE,
	T_DRIFT_X,
	T_DRIFT_Y,
	T_RMSE_X,
	T_RMSE_Y,
	T_SNR,
	T_POSITION_RA,
	T_POSITION_DEC,
	T_OFFSET_X,
	T_OFFSET_Y,
	T_BUMP_RA_S,
	T_BUMP_DEC_S,
	T_BUMP_ACTIVE,
	T_DITHER_X,
	T_DITHER_Y,
	T_SATURATED,
	T_LOST,
	T_DARK_FRAMES,
	T_DARK_ACTIVE,
	T_COUNT
};

static const setting_definition stats_definitions[T_COUNT] = {
	{ "PHASE", "Phase", -10, 100, 0, 0 },
	{ "FRAME_RATE", "Frame rate (Hz)", 0, 10000, 0, 0 },
	{ "DRIFT_X", "Residual X (px)", -10000, 10000, 0, 0 },
	{ "DRIFT_Y", "Residual Y (px)", -10000, 10000, 0, 0 },
	{ "RMSE_X", "RMSE X (px)", 0, 10000, 0, 0 },
	{ "RMSE_Y", "RMSE Y (px)", 0, 10000, 0, 0 },
	{ "SNR", "SNR", 0, 1000000, 0, 0 },
	{ "POSITION_RA", "AO position RA", -10000, 10000, 0, 0 },
	{ "POSITION_DEC", "AO position Dec", -10000, 10000, 0, 0 },
	{ "OFFSET_X", "AO deflection X (px)", -10000, 10000, 0, 0 },
	{ "OFFSET_Y", "AO deflection Y (px)", -10000, 10000, 0, 0 },
	{ "BUMP_RA_S", "Mount bump RA (arcsec)", -100000, 100000, 0, 0 },
	{ "BUMP_DEC_S", "Mount bump Dec (arcsec)", -100000, 100000, 0, 0 },
	{ "BUMP_ACTIVE", "Mount bump active", 0, 1, 0, 0 },
	{ "DITHER_X", "Dither X (px)", -10000, 10000, 0, 0 },
	{ "DITHER_Y", "Dither Y (px)", -10000, 10000, 0, 0 },
	{ "SATURATED", "AO at its limit", 0, 1, 0, 0 },
	{ "LOST_FRAMES", "Frames without star", 0, 1000000, 0, 0 },
	{ "DARK_FRAMES", "Frames in the dark frame", 0, MAX_DARK_FRAMES, 0, 0 },
	{ "DARK_ACTIVE", "Dark frame applied", 0, 1, 0, 0 }
};

// -------------------------------------------------------------------------------- private data

typedef struct {
	indigo_property *start_process_property;
	indigo_property *abort_process_property;
	indigo_property *mode_property;
	indigo_property *bump_mode_property;
	indigo_property *dec_mode_property;
	indigo_property *settings_property;
	indigo_property *stats_property;
	indigo_property *preview_property;
	pthread_mutex_t frame_mutex;
	char *frame_data;
	long frame_size;
	long frame_capacity;
	char frame_url[INDIGO_VALUE_SIZE];
	volatile unsigned long frame_serial;
	char *work_data;
	long work_size;
	long work_capacity;
	char *dark_data;
	double dark_exposure;
	int dark_count;
	unsigned dark_pedestal;
	bool dark_warned;
	char *preview_data;
	long preview_capacity;
	double preview_time;
	bool frames_wanted;
	bool streaming;
	bool has_streaming;
	volatile indigo_property_state exposure_state;
	volatile indigo_property_state streaming_state;
	int saved_upload_mode;
	int saved_image_format;
	volatile indigo_property_state ao_ra_state, ao_dec_state;
	volatile unsigned ao_ra_serial, ao_dec_serial;
	volatile indigo_property_state guide_ra_state, guide_dec_state;
	volatile unsigned guide_ra_serial, guide_dec_serial;
	double pos_ra, pos_dec;
	double want_ra, want_dec;
	bool saturated;
	bool locked;
	double star_x, star_y;
	int lock_width, lock_height;
	indigo_frame_digest reference;
	indigo_star_detection stars[MAX_STARS];
	int lost_frames;
	double snr;
	double rmse_x_sum, rmse_y_sum;
	unsigned long rmse_count;
	double stats_time;
	int rate_frames;
	double mount_dec;
	int mount_sop;
	volatile bool mount_slewing;
	volatile double guider_dithering;
	volatile indigo_property_state guider_start_state;
	volatile indigo_property_state guider_dither_state;
	bool bump_active;
	double dither_x, dither_y;
	int skip_frames;
	unsigned dither_seen, dither_count;
	volatile unsigned imager_exposures;
	volatile indigo_property_state imager_exposure_state;
	volatile indigo_property_state imager_start_state;
	double bump_x, bump_y;
	double done_x, done_y;
	double bump_time;
	double bump_pulse_time;
	int process;
	pthread_mutex_t mutex;
} ao_agent_private_data;

#define AGENT_START_PROCESS_PROPERTY					(DEVICE_PRIVATE_DATA->start_process_property)
#define AGENT_AO_CALIBRATION_ITEM							(AGENT_START_PROCESS_PROPERTY->items + 0)
#define AGENT_AO_BUMP_CALIBRATION_ITEM				(AGENT_START_PROCESS_PROPERTY->items + 1)
#define AGENT_AO_EXERCISE_ITEM								(AGENT_START_PROCESS_PROPERTY->items + 2)
#define AGENT_AO_CENTER_ITEM									(AGENT_START_PROCESS_PROPERTY->items + 3)
#define AGENT_AO_TRACKING_ITEM								(AGENT_START_PROCESS_PROPERTY->items + 4)
#define AGENT_AO_DARK_FRAME_ITEM							(AGENT_START_PROCESS_PROPERTY->items + 5)
#define AGENT_AO_DARK_RESET_ITEM							(AGENT_START_PROCESS_PROPERTY->items + 6)
#define AGENT_ABORT_PROCESS_PROPERTY					(DEVICE_PRIVATE_DATA->abort_process_property)
#define AGENT_ABORT_PROCESS_ITEM							(AGENT_ABORT_PROCESS_PROPERTY->items + 0)
#define AGENT_AO_MODE_PROPERTY								(DEVICE_PRIVATE_DATA->mode_property)
#define AGENT_AO_MODE_RELATIVE_ITEM						(AGENT_AO_MODE_PROPERTY->items + 0)
#define AGENT_AO_MODE_ABSOLUTE_ITEM						(AGENT_AO_MODE_PROPERTY->items + 1)
#define AGENT_AO_BUMP_MODE_PROPERTY						(DEVICE_PRIVATE_DATA->bump_mode_property)
#define AGENT_AO_BUMP_MODE_DIRECT_ITEM				(AGENT_AO_BUMP_MODE_PROPERTY->items + 0)
#define AGENT_AO_BUMP_MODE_OFF_ITEM						(AGENT_AO_BUMP_MODE_PROPERTY->items + 1)
#define AGENT_AO_DEC_MODE_PROPERTY						(DEVICE_PRIVATE_DATA->dec_mode_property)
#define AGENT_AO_DEC_MODE_BOTH_ITEM						(AGENT_AO_DEC_MODE_PROPERTY->items + 0)
#define AGENT_AO_DEC_MODE_NORTH_ITEM					(AGENT_AO_DEC_MODE_PROPERTY->items + 1)
#define AGENT_AO_DEC_MODE_SOUTH_ITEM					(AGENT_AO_DEC_MODE_PROPERTY->items + 2)
#define AGENT_AO_SETTINGS_PROPERTY						(DEVICE_PRIVATE_DATA->settings_property)
#define AGENT_AO_STATS_PROPERTY								(DEVICE_PRIVATE_DATA->stats_property)
#define AGENT_AO_PREVIEW_PROPERTY							(DEVICE_PRIVATE_DATA->preview_property)
#define AGENT_AO_PREVIEW_ITEM									(AGENT_AO_PREVIEW_PROPERTY->items + 0)
#define SET(n)																(AGENT_AO_SETTINGS_PROPERTY->items[n].number.value)
#define STAT(n)																(AGENT_AO_STATS_PROPERTY->items[n].number.value)

// -------------------------------------------------------------------------------- helpers

static inline bool aborted(indigo_device *device) {
	return AGENT_ABORT_PROCESS_PROPERTY->state == INDIGO_BUSY_STATE;
}

static inline bool ao_absolute(indigo_device *device) {
	return AGENT_AO_MODE_ABSOLUTE_ITEM->sw.value;
}

static inline double limit_to(double value, double limit) {
	return value > limit ? limit : (value < -limit ? -limit : value);
}

// The deflection of the AO (the distance of its position from the centre, in steps or percent) is limited to keep the tilt of the optics under control
static inline double deflection_limit(indigo_device *device) {
	return SET(S_RANGE);
}

static void save_config(indigo_device *device) {
	if (pthread_mutex_trylock(&DEVICE_CONTEXT->config_mutex) == 0) {
		pthread_mutex_unlock(&DEVICE_CONTEXT->config_mutex);
		pthread_mutex_lock(&DEVICE_PRIVATE_DATA->mutex);
		indigo_save_property(device, NULL, AGENT_AO_SETTINGS_PROPERTY);
		indigo_save_property(device, NULL, AGENT_AO_MODE_PROPERTY);
		indigo_save_property(device, NULL, AGENT_AO_BUMP_MODE_PROPERTY);
		indigo_save_property(device, NULL, AGENT_AO_DEC_MODE_PROPERTY);
		if (DEVICE_CONTEXT->property_save_file_handle != NULL) {
			CONFIG_PROPERTY->state = INDIGO_OK_STATE;
			indigo_uni_close(&DEVICE_CONTEXT->property_save_file_handle);
		} else {
			CONFIG_PROPERTY->state = INDIGO_ALERT_STATE;
		}
		CONFIG_SAVE_ITEM->sw.value = false;
		indigo_update_property(device, CONFIG_PROPERTY, NULL);
		pthread_mutex_unlock(&DEVICE_PRIVATE_DATA->mutex);
	}
}

static void update_stats(indigo_device *device) {
	STAT(T_POSITION_RA) = round(1000 * DEVICE_PRIVATE_DATA->pos_ra) / 1000;
	STAT(T_POSITION_DEC) = round(1000 * DEVICE_PRIVATE_DATA->pos_dec) / 1000;
	STAT(T_SATURATED) = DEVICE_PRIVATE_DATA->saturated ? 1 : 0;
	STAT(T_LOST) = DEVICE_PRIVATE_DATA->lost_frames;
	STAT(T_BUMP_ACTIVE) = DEVICE_PRIVATE_DATA->bump_active ? 1 : 0;
	STAT(T_DITHER_X) = round(1000 * DEVICE_PRIVATE_DATA->dither_x) / 1000;
	STAT(T_DITHER_Y) = round(1000 * DEVICE_PRIVATE_DATA->dither_y) / 1000;
	indigo_update_property(device, AGENT_AO_STATS_PROPERTY, NULL);
}

static void set_phase(indigo_device *device, int phase) {
	DEVICE_PRIVATE_DATA->process = phase;
	STAT(T_PHASE) = phase;
	update_stats(device);
}

static double get_pixel_scale(indigo_device *device) {
	return 3600 * sqrt(CCD_LENS_FOV_PIXEL_SCALE_WIDTH_ITEM->number.value * CCD_LENS_FOV_PIXEL_SCALE_HEIGHT_ITEM->number.value);
}

// -------------------------------------------------------------------------------- response model of the AO and of the mount

// The response of the star to a signed request (AO steps or guide pulse seconds) along two axes. Every direction of every axis has a vector of its own,
// so the positive and the negative direction of an axis need not be collinear or of the same scale, and the axes need not be orthogonal.
// The response is piecewise linear: a positive request uses the positive vector of its axis, a negative request the negative vector.
typedef struct {
	double pos[2][2]; // [axis][x, y] displacement per unit of a positive request
	double neg[2][2]; // [axis][x, y] displacement per unit of the magnitude of a negative request
} response_model;

#define MODEL_MIN_VECTOR 1e-9
#define MODEL_MIN_GAP 20
#define MODEL_MAX_GAP 160

static void model_forward(const response_model *model, double a, double b, double *x, double *y) {
	const double *va = a >= 0 ? model->pos[0] : model->neg[0];
	const double *vb = b >= 0 ? model->pos[1] : model->neg[1];
	*x = fabs(a) * va[0] + fabs(b) * vb[0];
	*y = fabs(a) * va[1] + fabs(b) * vb[1];
}

// The request that makes the displacement: the quadrant is the pair of neighbouring vectors whose cone holds the displacement, there both magnitudes are not negative
static void model_inverse(const response_model *model, double x, double y, double *a, double *b) {
	double best = -INFINITY;
	*a = *b = 0;
	for (int i = 0; i < 4; i++) {
		int neg_a = i & 1;
		int neg_b = i >> 1;
		const double *va = neg_a ? model->neg[0] : model->pos[0];
		const double *vb = neg_b ? model->neg[1] : model->pos[1];
		double det = va[0] * vb[1] - va[1] * vb[0];
		if (fabs(det) < MODEL_MIN_VECTOR) {
			continue;
		}
		double ma = (x * vb[1] - y * vb[0]) / det;
		double mb = (va[0] * y - va[1] * x) / det;
		if (fmin(ma, mb) > best) {
			best = fmin(ma, mb);
			*a = neg_a ? -fmax(ma, 0) : fmax(ma, 0);
			*b = neg_b ? -fmax(mb, 0) : fmax(mb, 0);
		}
	}
}

// the length of the vector that applies to a request
static double model_length(const response_model *model, int axis, double request) {
	const double *v = request >= 0 ? model->pos[axis] : model->neg[axis];
	return hypot(v[0], v[1]);
}

// The vectors +a, +b, -a, -b must follow each other around the circle in one sense with a gap between the neighbours that is neither too small nor 180 degrees or more,
// otherwise the cones do not tile the plane and the model has no inverse
static bool model_valid(const response_model *model) {
	const double *v[4] = { model->pos[0], model->pos[1], model->neg[0], model->neg[1] };
	double angle[4];
	for (int i = 0; i < 4; i++) {
		if (hypot(v[i][0], v[i][1]) < MODEL_MIN_VECTOR) {
			return false;
		}
		angle[i] = atan2(v[i][1], v[i][0]);
	}
	for (int sense = 1; sense >= -1; sense -= 2) {
		bool ok = true;
		for (int i = 0; i < 4 && ok; i++) {
			double gap = fmod(sense * (angle[(i + 1) % 4] - angle[i]), 2 * M_PI);
			if (gap < 0) {
				gap += 2 * M_PI;
			}
			gap = 180 * gap / M_PI;
			ok = gap >= MODEL_MIN_GAP && gap <= MODEL_MAX_GAP;
		}
		if (ok) {
			return true;
		}
	}
	return false;
}

// -------------------------------------------------------------------------------- AO model

static void ao_model(indigo_device *device, response_model *model) {
	model->pos[0][0] = SET(S_RA_X);
	model->pos[0][1] = SET(S_RA_Y);
	model->pos[1][0] = SET(S_DEC_X);
	model->pos[1][1] = SET(S_DEC_Y);
	model->neg[0][0] = SET(S_RA_MINUS_X);
	model->neg[0][1] = SET(S_RA_MINUS_Y);
	model->neg[1][0] = SET(S_DEC_MINUS_X);
	model->neg[1][1] = SET(S_DEC_MINUS_Y);
}

static bool ao_calibrated(indigo_device *device) {
	response_model model;
	ao_model(device, &model);
	return model_valid(&model);
}

// star displacement caused by an AO deflection
static void steps_to_px(indigo_device *device, double ra, double dec, double *x, double *y) {
	response_model model;
	ao_model(device, &model);
	model_forward(&model, ra, dec, x, y);
}

// AO deflection that moves the star by a displacement
static void px_to_steps(indigo_device *device, double x, double y, double *ra, double *dec) {
	response_model model;
	ao_model(device, &model);
	model_inverse(&model, x, y, ra, dec);
}

// -------------------------------------------------------------------------------- mount model

// the image turns by 180 degrees after a meridian flip, like the angle of the Guider Agent
static bool oag_flipped(indigo_device *device) {
	int calibrated = (int)SET(S_OAG_SOP);
	return calibrated != 0 && DEVICE_PRIVATE_DATA->mount_sop != 0 && calibrated != DEVICE_PRIVATE_DATA->mount_sop;
}

// The star displacement per second of a west, east, north and south guide pulse at the declination with the cosine cos_dec. The RA vectors are stored for the equator.
static bool mount_model(indigo_device *device, double cos_dec, response_model *model) {
	double v[8] = { SET(S_WEST_X), SET(S_WEST_Y), SET(S_EAST_X), SET(S_EAST_Y), SET(S_NORTH_X), SET(S_NORTH_Y), SET(S_SOUTH_X), SET(S_SOUTH_Y) };
	response_model result = { { { v[0] * cos_dec, v[1] * cos_dec }, { v[4], v[5] } }, { { v[2] * cos_dec, v[3] * cos_dec }, { v[6], v[7] } } };
	if (!model_valid(&result)) {
		return false;
	}
	if (oag_flipped(device)) {
		for (int axis = 0; axis < 2; axis++) {
			for (int k = 0; k < 2; k++) {
				result.pos[axis][k] = -result.pos[axis][k];
				result.neg[axis][k] = -result.neg[axis][k];
			}
		}
	}
	*model = result;
	return true;
}

static bool mount_calibrated(indigo_device *device) {
	response_model model;
	return mount_model(device, 1, &model);
}

// -------------------------------------------------------------------------------- frames

static void frames_reset(indigo_device *device) {
	DEVICE_PRIVATE_DATA->locked = false;
	DEVICE_PRIVATE_DATA->lost_frames = 0;
	DEVICE_PRIVATE_DATA->rmse_x_sum = DEVICE_PRIVATE_DATA->rmse_y_sum = 0;
	DEVICE_PRIVATE_DATA->rmse_count = 0;
}

static bool start_frames(indigo_device *device) {
	if (!INDIGO_FILTER_CCD_SELECTED) {
		indigo_send_message(device, ALERT_PROPERTY, "No camera selected");
		return false;
	}
	pthread_mutex_lock(&DEVICE_PRIVATE_DATA->frame_mutex);
	DEVICE_PRIVATE_DATA->frames_wanted = true;
	pthread_mutex_unlock(&DEVICE_PRIVATE_DATA->frame_mutex);
	DEVICE_PRIVATE_DATA->saved_upload_mode = indigo_save_switch_state(device, CCD_UPLOAD_MODE_PROPERTY_NAME, CCD_UPLOAD_MODE_CLIENT_ITEM_NAME);
	DEVICE_PRIVATE_DATA->saved_image_format = indigo_save_switch_state(device, CCD_IMAGE_FORMAT_PROPERTY_NAME, CCD_IMAGE_FORMAT_RAW_ITEM_NAME);
	DEVICE_PRIVATE_DATA->streaming = false;
	DEVICE_PRIVATE_DATA->dark_warned = false;
	if (DEVICE_PRIVATE_DATA->has_streaming) {
		static const char *names[] = { CCD_STREAMING_EXPOSURE_ITEM_NAME, CCD_STREAMING_COUNT_ITEM_NAME };
		double values[] = { SET(S_EXPOSURE), -1 };
		DEVICE_PRIVATE_DATA->streaming_state = INDIGO_IDLE_STATE;
		indigo_change_number_property(FILTER_DEVICE_CONTEXT->client, device->name, CCD_STREAMING_PROPERTY_NAME, 2, names, values);
		for (int i = 0; i < 3000 && DEVICE_PRIVATE_DATA->streaming_state != INDIGO_BUSY_STATE && !aborted(device); i++) {
			indigo_usleep(1000);
		}
		DEVICE_PRIVATE_DATA->streaming = DEVICE_PRIVATE_DATA->streaming_state == INDIGO_BUSY_STATE;
		if (!DEVICE_PRIVATE_DATA->streaming) {
			indigo_send_message(device, BUSY_PROPERTY, "Streaming did not start, using single exposures");
		}
	}
	frames_reset(device);
	return true;
}

static void stop_frames(indigo_device *device) {
	pthread_mutex_lock(&DEVICE_PRIVATE_DATA->frame_mutex);
	bool started = DEVICE_PRIVATE_DATA->frames_wanted;
	DEVICE_PRIVATE_DATA->frames_wanted = false;
	pthread_mutex_unlock(&DEVICE_PRIVATE_DATA->frame_mutex);
	if (!started) {
		return;
	}
	if (DEVICE_PRIVATE_DATA->streaming) {
		indigo_change_switch_property_1(FILTER_DEVICE_CONTEXT->client, device->name, CCD_ABORT_EXPOSURE_PROPERTY_NAME, CCD_ABORT_EXPOSURE_ITEM_NAME, true);
		for (int i = 0; i < 3000 && DEVICE_PRIVATE_DATA->streaming_state == INDIGO_BUSY_STATE; i++) {
			indigo_usleep(1000);
		}
		DEVICE_PRIVATE_DATA->streaming = false;
	}
	indigo_restore_switch_state(device, CCD_UPLOAD_MODE_PROPERTY_NAME, DEVICE_PRIVATE_DATA->saved_upload_mode);
	indigo_restore_switch_state(device, CCD_IMAGE_FORMAT_PROPERTY_NAME, DEVICE_PRIVATE_DATA->saved_image_format);
}

// Waits for the next image, copies it to the work buffer and validates it
static bool get_raw_frame(indigo_device *device, double timeout) {
	unsigned long serial = DEVICE_PRIVATE_DATA->frame_serial;
	if (!DEVICE_PRIVATE_DATA->streaming) {
		indigo_change_number_property_1(FILTER_DEVICE_CONTEXT->client, device->name, CCD_EXPOSURE_PROPERTY_NAME, CCD_EXPOSURE_ITEM_NAME, SET(S_EXPOSURE));
	}
	double deadline = indigo_monotonic_time() + timeout + (DEVICE_PRIVATE_DATA->streaming ? 0 : SET(S_EXPOSURE));
	while (DEVICE_PRIVATE_DATA->frame_serial == serial) {
		if (aborted(device) || indigo_monotonic_time() > deadline) {
			return false;
		}
		indigo_usleep(1000);
	}
	pthread_mutex_lock(&DEVICE_PRIVATE_DATA->frame_mutex);
	if (DEVICE_PRIVATE_DATA->frame_url[0]) {
		char url[INDIGO_VALUE_SIZE];
		INDIGO_COPY_VALUE(url, DEVICE_PRIVATE_DATA->frame_url);
		pthread_mutex_unlock(&DEVICE_PRIVATE_DATA->frame_mutex);
		void *data = NULL;
		long size = 0;
		if (!indigo_download_blob(url, &data, &size, NULL) || data == NULL) {
			indigo_safe_free(data);
			return false;
		}
		if (size > DEVICE_PRIVATE_DATA->work_capacity) {
			DEVICE_PRIVATE_DATA->work_data = indigo_safe_realloc(DEVICE_PRIVATE_DATA->work_data, size);
			DEVICE_PRIVATE_DATA->work_capacity = size;
		}
		memcpy(DEVICE_PRIVATE_DATA->work_data, data, size);
		DEVICE_PRIVATE_DATA->work_size = size;
		free(data);
	} else {
		if (DEVICE_PRIVATE_DATA->frame_size > DEVICE_PRIVATE_DATA->work_capacity) {
			DEVICE_PRIVATE_DATA->work_data = indigo_safe_realloc(DEVICE_PRIVATE_DATA->work_data, DEVICE_PRIVATE_DATA->frame_size);
			DEVICE_PRIVATE_DATA->work_capacity = DEVICE_PRIVATE_DATA->frame_size;
		}
		memcpy(DEVICE_PRIVATE_DATA->work_data, DEVICE_PRIVATE_DATA->frame_data, DEVICE_PRIVATE_DATA->frame_size);
		DEVICE_PRIVATE_DATA->work_size = DEVICE_PRIVATE_DATA->frame_size;
		pthread_mutex_unlock(&DEVICE_PRIVATE_DATA->frame_mutex);
	}
	indigo_raw_header *header = (indigo_raw_header *)DEVICE_PRIVATE_DATA->work_data;
	if (header == NULL || DEVICE_PRIVATE_DATA->work_size < (long)sizeof(indigo_raw_header)) {
		return false;
	}
	size_t bytes_per_pixel;
	switch (header->signature) {
		case INDIGO_RAW_MONO8: bytes_per_pixel = 1; break;
		case INDIGO_RAW_MONO16: bytes_per_pixel = 2; break;
		case INDIGO_RAW_RGB24: bytes_per_pixel = 3; break;
		case INDIGO_RAW_RGB48: bytes_per_pixel = 6; break;
		default: return false;
	}
	if (header->width == 0 || header->height == 0 || header->width > INT_MAX / bytes_per_pixel || header->height > INT_MAX / bytes_per_pixel / header->width) {
		return false;
	}
	size_t pixel_bytes = (size_t)header->width * header->height * bytes_per_pixel;
	return pixel_bytes <= (size_t)DEVICE_PRIVATE_DATA->work_size - sizeof(indigo_raw_header);
}

// -------------------------------------------------------------------------------- dark frame

// The samples of an image are 8 or 16 bit, an RGB pixel has three of them
static bool raw_layout(uint32_t signature, int *channels, int *sample_size) {
	switch (signature) {
		case INDIGO_RAW_MONO8: *channels = 1; *sample_size = 1; return true;
		case INDIGO_RAW_MONO16: *channels = 1; *sample_size = 2; return true;
		case INDIGO_RAW_RGB24: *channels = 3; *sample_size = 1; return true;
		case INDIGO_RAW_RGB48: *channels = 3; *sample_size = 2; return true;
	}
	return false;
}

static size_t raw_samples(const indigo_raw_header *header) {
	int channels, sample_size;
	return raw_layout(header->signature, &channels, &sample_size) ? (size_t)header->width * header->height * channels : 0;
}

// Median of a few values, the mean of the two middle ones when there is an even number of them
static unsigned median_of(unsigned *values, int count) {
	for (int i = 1; i < count; i++) {
		unsigned value = values[i];
		int j = i - 1;
		while (j >= 0 && values[j] > value) {
			values[j + 1] = values[j];
			j--;
		}
		values[j + 1] = value;
	}
	return count % 2 ? values[count / 2] : (values[count / 2 - 1] + values[count / 2] + 1) / 2;
}

// Combines count images of the same size (one after another in frames, samples samples each) into their per sample median
static void median_combine(const void *frames, size_t samples, int sample_size, int count, void *result) {
	unsigned values[MAX_DARK_FRAMES];
	for (size_t i = 0; i < samples; i++) {
		for (int j = 0; j < count; j++) {
			size_t index = (size_t)j * samples + i;
			values[j] = sample_size == 1 ? ((const uint8_t *)frames)[index] : ((const uint16_t *)frames)[index];
		}
		unsigned median = median_of(values, count);
		if (sample_size == 1) {
			((uint8_t *)result)[i] = (uint8_t)median;
		} else {
			((uint16_t *)result)[i] = (uint16_t)median;
		}
	}
}

// Noise (sigma of a normal distribution) of the samples of the frames around their median. It is estimated from the mean absolute
// deviation, corrected for the median coming from the same few samples. Outliers such as cosmic rays move it little.
static double noise_around_median(const void *frames, size_t samples, int sample_size, int count, const void *median) {
	double sum = 0;
	for (size_t i = 0; i < samples; i++) {
		double m = sample_size == 1 ? ((const uint8_t *)median)[i] : ((const uint16_t *)median)[i];
		for (int j = 0; j < count; j++) {
			size_t index = (size_t)j * samples + i;
			double v = sample_size == 1 ? ((const uint8_t *)frames)[index] : ((const uint16_t *)frames)[index];
			sum += fabs(v - m);
		}
	}
	return 1.2533 * sqrt((double)count / (count - 1)) * sum / ((double)samples * count);
}

// Pedestal that keeps the noise of a dark subtracted image above zero: four sigma of the noise of the dark frames, which is at most 1/16 of the range
static unsigned dark_pedestal_for(double noise, int sample_size) {
	unsigned limit = sample_size == 1 ? 255 / 16 : 65535 / 16;
	double pedestal = ceil(4 * noise);
	return pedestal > limit ? limit : (unsigned)pedestal;
}

// Subtracts the dark samples from the image and adds the pedestal, so that the noise of the background is not cut at zero. The result stays within the range of the samples.
static void subtract_dark(void *pixels, const void *dark, size_t samples, int sample_size, unsigned pedestal) {
	if (sample_size == 1) {
		uint8_t *p = (uint8_t *)pixels;
		const uint8_t *d = (const uint8_t *)dark;
		for (size_t i = 0; i < samples; i++) {
			int value = (int)p[i] - d[i] + (int)pedestal;
			p[i] = value < 0 ? 0 : (value > UINT8_MAX ? UINT8_MAX : value);
		}
	} else {
		uint16_t *p = (uint16_t *)pixels;
		const uint16_t *d = (const uint16_t *)dark;
		for (size_t i = 0; i < samples; i++) {
			int value = (int)p[i] - d[i] + (int)pedestal;
			p[i] = value < 0 ? 0 : (value > UINT16_MAX ? UINT16_MAX : value);
		}
	}
}

static void dark_clear(indigo_device *device) {
	indigo_safe_free(DEVICE_PRIVATE_DATA->dark_data);
	DEVICE_PRIVATE_DATA->dark_data = NULL;
	DEVICE_PRIVATE_DATA->dark_count = 0;
	DEVICE_PRIVATE_DATA->dark_pedestal = 0;
	DEVICE_PRIVATE_DATA->dark_exposure = 0;
	STAT(T_DARK_FRAMES) = 0;
	STAT(T_DARK_ACTIVE) = 0;
}

// The dark frame fits when it was taken with the exposure and the frame (size, binning, pixel format) of the image
static bool dark_fits(indigo_device *device, const indigo_raw_header *header) {
	const indigo_raw_header *dark = (const indigo_raw_header *)DEVICE_PRIVATE_DATA->dark_data;
	return dark != NULL && dark->signature == header->signature && dark->width == header->width && dark->height == header->height && fabs(DEVICE_PRIVATE_DATA->dark_exposure - SET(S_EXPOSURE)) < 1e-6;
}

// The dark frame is optional, an image is used as it is without one or when it does not fit
static void dark_apply(indigo_device *device) {
	indigo_raw_header *header = (indigo_raw_header *)DEVICE_PRIVATE_DATA->work_data;
	bool active = false;
	if (DEVICE_PRIVATE_DATA->dark_data != NULL) {
		if (dark_fits(device, header)) {
			int channels, sample_size;
			if (raw_layout(header->signature, &channels, &sample_size)) {
				subtract_dark(header + 1, (indigo_raw_header *)DEVICE_PRIVATE_DATA->dark_data + 1, raw_samples(header), sample_size, DEVICE_PRIVATE_DATA->dark_pedestal);
				active = true;
			}
		} else if (!DEVICE_PRIVATE_DATA->dark_warned) {
			DEVICE_PRIVATE_DATA->dark_warned = true;
			indigo_send_message(device, BUSY_PROPERTY, "The dark frame was taken with another exposure or frame and is not used, take a new one or reset it");
		}
	}
	STAT(T_DARK_ACTIVE) = active ? 1 : 0;
}

// Publishes the image the star positions are measured on (with the dark frame subtracted) as a stretched JPEG, once per PREVIEW_PERIOD
static void preview_publish(indigo_device *device) {
	double now = indigo_monotonic_time();
	if (now - DEVICE_PRIVATE_DATA->preview_time < PREVIEW_PERIOD) {
		return;
	}
	DEVICE_PRIVATE_DATA->preview_time = now;
	indigo_raw_header *header = (indigo_raw_header *)DEVICE_PRIVATE_DATA->work_data;
	int channels, sample_size;
	if (!raw_layout(header->signature, &channels, &sample_size)) {
		return;
	}
	void *jpeg = NULL;
	unsigned long size = 0;
	indigo_raw_to_jpeg(device, header + 1, (int)header->width, (int)header->height, 8 * channels * sample_size, NULL, &jpeg, &size, NULL, NULL, ccd_jpeg_stretch_params_lut[CCD_JPEG_STRETCH_NORMAL].target_background, ccd_jpeg_stretch_params_lut[CCD_JPEG_STRETCH_NORMAL].clipping_point, 0);
	if (jpeg == NULL || size == 0) {
		indigo_safe_free(jpeg);
		return;
	}
	if ((long)size > DEVICE_PRIVATE_DATA->preview_capacity) {
		DEVICE_PRIVATE_DATA->preview_data = indigo_safe_realloc(DEVICE_PRIVATE_DATA->preview_data, size);
		DEVICE_PRIVATE_DATA->preview_capacity = size;
	}
	memcpy(DEVICE_PRIVATE_DATA->preview_data, jpeg, size);
	free(jpeg);
	AGENT_AO_PREVIEW_ITEM->blob.value = DEVICE_PRIVATE_DATA->preview_data;
	AGENT_AO_PREVIEW_ITEM->blob.size = size;
	INDIGO_COPY_NAME(AGENT_AO_PREVIEW_ITEM->blob.format, ".jpeg");
	AGENT_AO_PREVIEW_PROPERTY->state = INDIGO_OK_STATE;
	indigo_update_property(device, AGENT_AO_PREVIEW_PROPERTY, NULL);
}

// Waits for the next image and subtracts the dark frame from it
static bool get_frame(indigo_device *device, double timeout) {
	if (!get_raw_frame(device, timeout)) {
		return false;
	}
	dark_apply(device);
	preview_publish(device);
	return true;
}

// -------------------------------------------------------------------------------- tracker

// Displacement of the star against the reference, locks on the brightest star of the first frame
static bool analyze_frame(indigo_device *device, double *dx, double *dy) {
	indigo_raw_header *header = (indigo_raw_header *)DEVICE_PRIVATE_DATA->work_data;
	char *data = (char *)header + sizeof(indigo_raw_header);
	int radius = (int)SET(S_RADIUS);
	int width = (int)header->width;
	int height = (int)header->height;
	if (DEVICE_PRIVATE_DATA->locked && (DEVICE_PRIVATE_DATA->lock_width != width || DEVICE_PRIVATE_DATA->lock_height != height)) {
		DEVICE_PRIVATE_DATA->locked = false;
	}
	if (!DEVICE_PRIVATE_DATA->locked) {
		int found = 0;
		if (indigo_find_stars_precise(header->signature, data, radius, width, height, MAX_STARS, DEVICE_PRIVATE_DATA->stars, &found) != INDIGO_OK || found == 0) {
			return false;
		}
		int best = -1;
		for (int i = 0; i < found; i++) {
			indigo_star_detection *star = DEVICE_PRIVATE_DATA->stars + i;
			if (star->oversaturated || star->close_to_other) {
				continue;
			}
			if (best < 0 || star->luminance > DEVICE_PRIVATE_DATA->stars[best].luminance) {
				best = i;
			}
		}
		if (best < 0) {
			best = 0;
		}
		DEVICE_PRIVATE_DATA->star_x = DEVICE_PRIVATE_DATA->stars[best].x;
		DEVICE_PRIVATE_DATA->star_y = DEVICE_PRIVATE_DATA->stars[best].y;
		if (indigo_selection_frame_digest_iterative(header->signature, data, &DEVICE_PRIVATE_DATA->star_x, &DEVICE_PRIVATE_DATA->star_y, radius, width, height, &DEVICE_PRIVATE_DATA->reference, 3) != INDIGO_OK) {
			return false;
		}
		DEVICE_PRIVATE_DATA->lock_width = width;
		DEVICE_PRIVATE_DATA->lock_height = height;
		DEVICE_PRIVATE_DATA->locked = true;
		DEVICE_PRIVATE_DATA->snr = DEVICE_PRIVATE_DATA->reference.snr;
		*dx = *dy = 0;
		return true;
	}
	indigo_frame_digest digest = { 0 };
	if (indigo_selection_frame_digest_iterative(header->signature, data, &DEVICE_PRIVATE_DATA->star_x, &DEVICE_PRIVATE_DATA->star_y, radius, width, height, &digest, 1) != INDIGO_OK) {
		indigo_delete_frame_digest(&digest);
		return false;
	}
	*dx = digest.centroid_x - DEVICE_PRIVATE_DATA->reference.centroid_x;
	*dy = digest.centroid_y - DEVICE_PRIVATE_DATA->reference.centroid_y;
	DEVICE_PRIVATE_DATA->snr = digest.snr;
	indigo_delete_frame_digest(&digest);
	return true;
}

// Mean displacement of several frames
static bool measure_average(indigo_device *device, int frames, double *x, double *y) {
	double sum_x = 0, sum_y = 0;
	for (int i = 0; i < frames; i++) {
		double dx, dy;
		if (!get_frame(device, FRAME_TIMEOUT) || !analyze_frame(device, &dx, &dy)) {
			return false;
		}
		sum_x += dx;
		sum_y += dy;
	}
	*x = sum_x / frames;
	*y = sum_y / frames;
	return true;
}

static void relock(indigo_device *device) {
	DEVICE_PRIVATE_DATA->locked = false;
	DEVICE_PRIVATE_DATA->lost_frames = 0;
}

// -------------------------------------------------------------------------------- AO

static indigo_property_state wait_update(volatile unsigned *serial, unsigned before, volatile indigo_property_state *state, double timeout, indigo_device *device) {
	double deadline = indigo_monotonic_time() + timeout;
	while (indigo_monotonic_time() < deadline) {
		if (*serial != before && *state != INDIGO_BUSY_STATE) {
			return *state;
		}
		indigo_usleep(1000);
	}
	return INDIGO_ALERT_STATE;
}

// One request to the AO axes, the values are steps (relative) or percent (absolute), positive along EAST and NORTH.
// A relative AO is not asked for a zero move, an absolute one centers an axis with a zero request.
static bool ao_request(indigo_device *device, double ra, double dec) {
	static const char *ra_names[] = { AO_GUIDE_EAST_ITEM_NAME, AO_GUIDE_WEST_ITEM_NAME };
	static const char *dec_names[] = { AO_GUIDE_NORTH_ITEM_NAME, AO_GUIDE_SOUTH_ITEM_NAME };
	double ra_values[] = { ra > 0 ? ra : 0, ra < 0 ? -ra : 0 };
	double dec_values[] = { dec > 0 ? dec : 0, dec < 0 ? -dec : 0 };
	bool ra_sent = ao_absolute(device) || ra != 0;
	bool dec_sent = ao_absolute(device) || dec != 0;
	unsigned ra_before = DEVICE_PRIVATE_DATA->ao_ra_serial;
	unsigned dec_before = DEVICE_PRIVATE_DATA->ao_dec_serial;
	if (ra_sent) {
		indigo_change_number_property(FILTER_DEVICE_CONTEXT->client, device->name, AO_GUIDE_RA_PROPERTY_NAME, 2, ra_names, ra_values);
	}
	if (dec_sent) {
		indigo_change_number_property(FILTER_DEVICE_CONTEXT->client, device->name, AO_GUIDE_DEC_PROPERTY_NAME, 2, dec_names, dec_values);
	}
	double timeout = SET(S_AO_TIMEOUT);
	indigo_property_state ra_state = ra_sent ? wait_update(&DEVICE_PRIVATE_DATA->ao_ra_serial, ra_before, &DEVICE_PRIVATE_DATA->ao_ra_state, timeout, device) : INDIGO_OK_STATE;
	indigo_property_state dec_state = dec_sent ? wait_update(&DEVICE_PRIVATE_DATA->ao_dec_serial, dec_before, &DEVICE_PRIVATE_DATA->ao_dec_state, timeout, device) : INDIGO_OK_STATE;
	if (ra_state == INDIGO_ALERT_STATE || dec_state == INDIGO_ALERT_STATE) {
		DEVICE_PRIVATE_DATA->saturated = true;
		return false;
	}
	return true;
}

// Moves the AO to a position. A relative AO is moved by integer steps, the position is the sum of the steps made.
static bool ao_move_to(indigo_device *device, double ra, double dec) {
	double limit = deflection_limit(device);
	double norm = hypot(ra, dec);
	if (norm > limit) {
		ra *= limit / norm;
		dec *= limit / norm;
	}
	DEVICE_PRIVATE_DATA->want_ra = ra;
	DEVICE_PRIVATE_DATA->want_dec = dec;
	bool ok = true;
	if (ao_absolute(device)) {
		if (fabs(ra - DEVICE_PRIVATE_DATA->pos_ra) < 1e-6 && fabs(dec - DEVICE_PRIVATE_DATA->pos_dec) < 1e-6) {
			return true;
		}
		ok = ao_request(device, ra, dec);
		DEVICE_PRIVATE_DATA->pos_ra = ra;
		DEVICE_PRIVATE_DATA->pos_dec = dec;
	} else {
		double max_step = SET(S_MAX_STEP);
		for (int i = 0; i < 1000; i++) {
			double delta_ra = round(ra - DEVICE_PRIVATE_DATA->pos_ra);
			double delta_dec = round(dec - DEVICE_PRIVATE_DATA->pos_dec);
			if (delta_ra == 0 && delta_dec == 0) {
				break;
			}
			delta_ra = limit_to(delta_ra, max_step);
			delta_dec = limit_to(delta_dec, max_step);
			ok = ao_request(device, delta_ra, delta_dec);
			DEVICE_PRIVATE_DATA->pos_ra += delta_ra;
			DEVICE_PRIVATE_DATA->pos_dec += delta_dec;
			if (!ok || aborted(device)) {
				break;
			}
		}
	}
	return ok;
}

static bool ao_center(indigo_device *device) {
	if (indigo_change_switch_property_1(FILTER_DEVICE_CONTEXT->client, device->name, AO_RESET_PROPERTY_NAME, AO_CENTER_ITEM_NAME, true) != INDIGO_OK) {
		return false;
	}
	indigo_usleep(50000);
	for (int i = 0; i < (int)(SET(S_AO_TIMEOUT) * 100) && (DEVICE_PRIVATE_DATA->ao_ra_state == INDIGO_BUSY_STATE || DEVICE_PRIVATE_DATA->ao_dec_state == INDIGO_BUSY_STATE); i++) {
		indigo_usleep(10000);
	}
	DEVICE_PRIVATE_DATA->pos_ra = DEVICE_PRIVATE_DATA->pos_dec = 0;
	DEVICE_PRIVATE_DATA->want_ra = DEVICE_PRIVATE_DATA->want_dec = 0;
	DEVICE_PRIVATE_DATA->dither_x = DEVICE_PRIVATE_DATA->dither_y = 0;
	DEVICE_PRIVATE_DATA->saturated = false;
	return true;
}

// -------------------------------------------------------------------------------- mount

static bool pulse_guide(indigo_device *device, double ra_seconds, double dec_seconds) {
	static const char *ra_names[] = { GUIDER_GUIDE_WEST_ITEM_NAME, GUIDER_GUIDE_EAST_ITEM_NAME };
	static const char *dec_names[] = { GUIDER_GUIDE_NORTH_ITEM_NAME, GUIDER_GUIDE_SOUTH_ITEM_NAME };
	double ra_values[] = { ra_seconds > 0 ? 1000 * ra_seconds : 0, ra_seconds < 0 ? -1000 * ra_seconds : 0 };
	double dec_values[] = { dec_seconds > 0 ? 1000 * dec_seconds : 0, dec_seconds < 0 ? -1000 * dec_seconds : 0 };
	unsigned ra_before = DEVICE_PRIVATE_DATA->guide_ra_serial;
	unsigned dec_before = DEVICE_PRIVATE_DATA->guide_dec_serial;
	bool ra_sent = ra_seconds != 0;
	bool dec_sent = dec_seconds != 0;
	if (ra_sent) {
		indigo_change_number_property(FILTER_DEVICE_CONTEXT->client, device->name, GUIDER_GUIDE_RA_PROPERTY_NAME, 2, ra_names, ra_values);
	}
	if (dec_sent) {
		indigo_change_number_property(FILTER_DEVICE_CONTEXT->client, device->name, GUIDER_GUIDE_DEC_PROPERTY_NAME, 2, dec_names, dec_values);
	}
	double timeout = fmax(fabs(ra_seconds), fabs(dec_seconds)) + 5;
	bool ok = true;
	if (ra_sent) {
		ok = wait_update(&DEVICE_PRIVATE_DATA->guide_ra_serial, ra_before, &DEVICE_PRIVATE_DATA->guide_ra_state, timeout, device) == INDIGO_OK_STATE;
	}
	if (dec_sent) {
		ok = (wait_update(&DEVICE_PRIVATE_DATA->guide_dec_serial, dec_before, &DEVICE_PRIVATE_DATA->guide_dec_state, timeout, device) == INDIGO_OK_STATE) && ok;
	}
	return ok;
}

static bool guider_agent_name(indigo_device *device, char *name) {
	char *related = indigo_filter_first_related_agent(device, GUIDER_AGENT_PREFIX);
	if (related == NULL) {
		return false;
	}
	INDIGO_COPY_NAME(name, related);
	return true;
}

// -------------------------------------------------------------------------------- bump

static bool guider_agent_related(indigo_device *device) {
	char name[INDIGO_NAME_SIZE];
	return guider_agent_name(device, name);
}

static bool guider_agent_busy(indigo_device *device) {
	char name[INDIGO_NAME_SIZE];
	return guider_agent_name(device, name) && DEVICE_PRIVATE_DATA->guider_start_state == INDIGO_BUSY_STATE;
}

// The Guider Agent is the only one to move the mount while it calibrates, previews or guides
static bool bump_wanted(indigo_device *device) {
	if (AGENT_AO_BUMP_MODE_OFF_ITEM->sw.value || !mount_calibrated(device) || DEVICE_PRIVATE_DATA->mount_slewing) {
		return false;
	}
	return !guider_agent_busy(device) && INDIGO_FILTER_GUIDER_SELECTED;
}

static void bump_reset(indigo_device *device) {
	DEVICE_PRIVATE_DATA->bump_x = DEVICE_PRIVATE_DATA->bump_y = 0;
	DEVICE_PRIVATE_DATA->done_x = DEVICE_PRIVATE_DATA->done_y = 0;
	DEVICE_PRIVATE_DATA->bump_time = 0;
	STAT(T_BUMP_RA_S) = STAT(T_BUMP_DEC_S) = 0;
}

// One step of the slow loop: the total displacement T integrates the deflection of the AO and is realised by guide pulses
static void bump_step(indigo_device *device, double dt) {
	bool active = bump_wanted(device);
	if (active != DEVICE_PRIVATE_DATA->bump_active) {
		DEVICE_PRIVATE_DATA->bump_active = active;
		indigo_send_message(device, IDLE_PROPERTY, active ? "Mount bump sends guide pulses" : "Mount bump is off");
	}
	if (!active) {
		return;
	}
	double a_x, a_y;
	steps_to_px(device, DEVICE_PRIVATE_DATA->pos_ra, DEVICE_PRIVATE_DATA->pos_dec, &a_x, &a_y);
	a_x -= DEVICE_PRIVATE_DATA->dither_x;
	a_y -= DEVICE_PRIVATE_DATA->dither_y;
	double factor = SET(S_BUMP_GAIN) * dt / SET(S_BUMP_TAU);
	DEVICE_PRIVATE_DATA->bump_x += a_x * factor;
	DEVICE_PRIVATE_DATA->bump_y += a_y * factor;
	double now = indigo_monotonic_time();
	double cos_dec = cos(M_PI * DEVICE_PRIVATE_DATA->mount_dec / 180);
	if (cos_dec < 0.05) {
		cos_dec = 0.05;
	}
	response_model model;
	if (!mount_model(device, cos_dec, &model)) {
		return;
	}
	double ra_s, dec_s;
	model_inverse(&model, DEVICE_PRIVATE_DATA->bump_x - DEVICE_PRIVATE_DATA->done_x, DEVICE_PRIVATE_DATA->bump_y - DEVICE_PRIVATE_DATA->done_y, &ra_s, &dec_s);
	// the displacement along the direction of each pulse in pixels
	double ra_px = ra_s * model_length(&model, 0, ra_s);
	double dec_px = dec_s * model_length(&model, 1, dec_s);
	STAT(T_BUMP_RA_S) = round(1000 * ra_px * get_pixel_scale(device)) / 1000;
	STAT(T_BUMP_DEC_S) = round(1000 * dec_px * get_pixel_scale(device)) / 1000;
	double threshold = SET(S_BUMP_THRESHOLD);
	if (now - DEVICE_PRIVATE_DATA->bump_pulse_time < SET(S_BUMP_INTERVAL) || (fabs(ra_px) < threshold && fabs(dec_px) < threshold)) {
		return;
	}
	if (fabs(ra_px) < threshold) {
		ra_s = 0;
	}
	if (fabs(dec_px) < threshold) {
		dec_s = 0;
	}
	if ((AGENT_AO_DEC_MODE_NORTH_ITEM->sw.value && dec_s < 0) || (AGENT_AO_DEC_MODE_SOUTH_ITEM->sw.value && dec_s > 0)) {
		dec_s = 0;
	}
	ra_s = limit_to(ra_s, SET(S_MAX_PULSE));
	dec_s = limit_to(dec_s, SET(S_MAX_PULSE));
	if (fabs(ra_s) < SET(S_MIN_PULSE)) {
		ra_s = 0;
	}
	if (fabs(dec_s) < SET(S_MIN_PULSE)) {
		dec_s = 0;
	}
	if (ra_s == 0 && dec_s == 0) {
		return;
	}
	// nothing but this agent moves the mount here, but a process of the Guider Agent may have started meanwhile
	if (guider_agent_busy(device)) {
		return;
	}
	DEVICE_PRIVATE_DATA->bump_pulse_time = now;
	if (pulse_guide(device, ra_s, dec_s)) {
		double x, y;
		model_forward(&model, ra_s, dec_s, &x, &y);
		DEVICE_PRIVATE_DATA->done_x += x;
		DEVICE_PRIVATE_DATA->done_y += y;
	}
}

// -------------------------------------------------------------------------------- processes

static void process_begin(indigo_device *device, int phase) {
	FILTER_DEVICE_CONTEXT->running_process = true;
	set_phase(device, phase);
}

static void process_end(indigo_device *device, bool ok, const char *message) {
	bool was_aborted = aborted(device);
	set_phase(device, was_aborted ? AGENT_AO_PHASE_IDLE : (ok ? AGENT_AO_PHASE_DONE : AGENT_AO_PHASE_FAILED));
	for (int i = 0; i < AGENT_START_PROCESS_PROPERTY->count; i++) {
		AGENT_START_PROCESS_PROPERTY->items[i].sw.value = false;
	}
	AGENT_START_PROCESS_PROPERTY->state = ok || was_aborted ? INDIGO_OK_STATE : INDIGO_ALERT_STATE;
	indigo_update_property(device, AGENT_START_PROCESS_PROPERTY, message);
	if (was_aborted) {
		AGENT_ABORT_PROCESS_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, AGENT_ABORT_PROCESS_PROPERTY, NULL);
	}
	FILTER_DEVICE_CONTEXT->running_process = false;
}

static bool ao_ready(indigo_device *device) {
	if (!INDIGO_FILTER_AO_SELECTED) {
		indigo_send_message(device, ALERT_PROPERTY, "No adaptive optics selected");
		return false;
	}
	return true;
}

static void center_process(indigo_device *device) {
	process_begin(device, AGENT_AO_PHASE_CENTERING);
	bool ok = ao_ready(device) && ao_center(device);
	bump_reset(device);
	process_end(device, ok, ok ? "AO centered" : "AO centering failed");
}

// Takes the dark frames with the exposure of the loop and keeps their median. The OAG must be covered, unless the camera closes its shutter for frame type DARK.
// The images the calibrations and the loop measure are corrected with this dark frame while it fits.
static void dark_process(indigo_device *device) {
	process_begin(device, AGENT_AO_PHASE_DARK);
	int count = (int)SET(S_DARK_FRAMES);
	if (count < MIN_DARK_FRAMES) {
		count = MIN_DARK_FRAMES;
	} else if (count > MAX_DARK_FRAMES) {
		count = MAX_DARK_FRAMES;
	}
	bool ok = INDIGO_FILTER_CCD_SELECTED;
	int saved_frame_type = -1;
	if (!ok) {
		indigo_send_message(device, ALERT_PROPERTY, "No camera selected");
	} else {
		saved_frame_type = indigo_save_switch_state(device, CCD_FRAME_TYPE_PROPERTY_NAME, CCD_FRAME_TYPE_DARK_ITEM_NAME);
		ok = start_frames(device);
	}
	char *stack = NULL;
	char *dark = NULL;
	size_t samples = 0, stack_bytes = 0;
	int sample_size = 0, taken = 0;
	indigo_raw_header first = { 0 };
	if (ok) {
		indigo_send_message(device, BUSY_PROPERTY, "Taking %d dark frames of %g s, the camera must be covered", count, SET(S_EXPOSURE));
	}
	// the first frames may still have been exposed before the frame type changed
	for (int i = -DARK_SKIP_FRAMES; ok && i < count && !aborted(device); i++) {
		if (!get_raw_frame(device, FRAME_TIMEOUT)) {
			if (!aborted(device)) {
				indigo_send_message(device, ALERT_PROPERTY, "No dark frame received from the camera");
				ok = false;
			}
			break;
		}
		if (i < 0) {
			continue;
		}
		indigo_raw_header *header = (indigo_raw_header *)DEVICE_PRIVATE_DATA->work_data;
		if (taken == 0) {
			int channels;
			first = *header;
			samples = raw_samples(header);
			raw_layout(header->signature, &channels, &sample_size);
			stack_bytes = samples * sample_size;
			stack = indigo_safe_malloc((size_t)count * stack_bytes);
		} else if (header->signature != first.signature || header->width != first.width || header->height != first.height) {
			indigo_send_message(device, ALERT_PROPERTY, "The frame changed while the dark frames were taken");
			ok = false;
			break;
		}
		memcpy(stack + (size_t)taken * stack_bytes, header + 1, stack_bytes);
		taken++;
		STAT(T_DARK_FRAMES) = taken;
		update_stats(device);
	}
	if (ok && !aborted(device) && taken == count) {
		dark = indigo_safe_malloc(sizeof(indigo_raw_header) + stack_bytes);
		memcpy(dark, &first, sizeof(indigo_raw_header));
		median_combine(stack, samples, sample_size, count, dark + sizeof(indigo_raw_header));
		double noise = noise_around_median(stack, samples, sample_size, count, dark + sizeof(indigo_raw_header));
		dark_clear(device);
		DEVICE_PRIVATE_DATA->dark_data = dark;
		DEVICE_PRIVATE_DATA->dark_count = count;
		DEVICE_PRIVATE_DATA->dark_pedestal = dark_pedestal_for(noise, sample_size);
		DEVICE_PRIVATE_DATA->dark_exposure = SET(S_EXPOSURE);
		indigo_send_message(device, OK_PROPERTY, "Dark frame of %d frames combined, %dx%d, exposure %g s, noise %.2f, pedestal %u", count, (int)first.width, (int)first.height, SET(S_EXPOSURE), noise, DEVICE_PRIVATE_DATA->dark_pedestal);
	}
	// a failed or aborted attempt keeps the previous dark frame
	STAT(T_DARK_FRAMES) = DEVICE_PRIVATE_DATA->dark_count;
	indigo_safe_free(stack);
	stop_frames(device);
	indigo_restore_switch_state(device, CCD_FRAME_TYPE_PROPERTY_NAME, saved_frame_type);
	process_end(device, ok, ok ? "Dark frame taken" : "Dark frame failed");
}

static void dark_reset_process(indigo_device *device) {
	process_begin(device, AGENT_AO_PHASE_DARK);
	dark_clear(device);
	process_end(device, true, "Dark frame reset");
}

// Visits the four corners and the centre of the range with an interval, the AO can be watched in the camera of the OAG
static void exercise_process(indigo_device *device) {
	process_begin(device, AGENT_AO_PHASE_EXERCISING);
	bool ok = ao_ready(device) && ao_center(device);
	// 100 % puts the corners at the maximum deflection
	double amplitude = deflection_limit(device) * SET(S_EX_AMPLITUDE) / 100 / sqrt(2);
	static const int corners[5][2] = { { 1, 1 }, { 1, -1 }, { -1, -1 }, { -1, 1 }, { 0, 0 } };
	int cycles = (int)SET(S_EX_CYCLES);
	for (int cycle = 0; ok && (cycles == 0 || cycle < cycles) && !aborted(device); cycle++) {
		for (int i = 0; ok && i < 5 && !aborted(device); i++) {
			ok = ao_move_to(device, corners[i][0] * amplitude, corners[i][1] * amplitude);
			update_stats(device);
			for (double waited = 0; waited < SET(S_EX_INTERVAL) && !aborted(device); waited += 0.05) {
				indigo_usleep(50000);
			}
		}
	}
	bool centered = ao_ready(device) && ao_center(device);
	process_end(device, ok && centered, ok ? "AO exercise finished" : "AO exercise failed");
}

// The AO is moved along both axes while the star is measured, the loop is open
static void calibrate_process(indigo_device *device) {
	process_begin(device, AGENT_AO_PHASE_CALIBRATING);
	bool ok = ao_ready(device) && start_frames(device) && ao_center(device);
	// the calibration moves must stay within the maximum deflection, the AO is not asked for more
	double step = fmin(SET(S_CAL_STEP), deflection_limit(device));
	// every direction is measured on its own: +RA, -RA, +Dec, -Dec. Each is a move from the centre and back, measured against the centre before and after it.
	static const double moves[4][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
	double vectors[4][2] = { { 0 } };
	int frames = (int)SET(S_CAL_FRAMES);
	for (int attempt = 0; ok && attempt < 4 && !aborted(device); attempt++) {
		double base[5][2], moved[4][2];
		double settle = SET(S_CAL_SETTLE);
		ok = measure_average(device, frames, &base[0][0], &base[0][1]);
		for (int i = 0; ok && i < 4; i++) {
			ok = ao_move_to(device, step * moves[i][0], step * moves[i][1]);
			indigo_usleep((long)(settle * 1e6));
			ok = ok && measure_average(device, frames, &moved[i][0], &moved[i][1]);
			ok = ok && ao_move_to(device, 0, 0);
			indigo_usleep((long)(settle * 1e6));
			ok = ok && measure_average(device, frames, &base[i + 1][0], &base[i + 1][1]);
		}
		if (!ok) {
			break;
		}
		bool weak = false;
		for (int i = 0; i < 4; i++) {
			vectors[i][0] = (moved[i][0] - (base[i][0] + base[i + 1][0]) / 2) / step;
			vectors[i][1] = (moved[i][1] - (base[i][1] + base[i + 1][1]) / 2) / step;
			weak = weak || hypot(vectors[i][0], vectors[i][1]) < 0.05;
		}
		if (weak) {
			if (step * 2 > deflection_limit(device)) {
				ok = false;
				indigo_send_message(device, ALERT_PROPERTY, "The star does not move with the AO, check the AO and the camera");
				break;
			}
			step *= 2;
			indigo_send_message(device, BUSY_PROPERTY, "Increasing the AO calibration step to %g", step);
			memset(vectors, 0, sizeof(vectors));
			continue;
		}
		break;
	}
	if (ok && !aborted(device)) {
		response_model model = { { { vectors[0][0], vectors[0][1] }, { vectors[2][0], vectors[2][1] } }, { { vectors[1][0], vectors[1][1] }, { vectors[3][0], vectors[3][1] } } };
		if (!model_valid(&model)) {
			ok = false;
			indigo_send_message(device, ALERT_PROPERTY, "The AO directions are not usable, the neighbouring ones must follow each other around the image %d to %d degrees apart", MODEL_MIN_GAP, MODEL_MAX_GAP);
		} else {
			SET(S_RA_X) = vectors[0][0];
			SET(S_RA_Y) = vectors[0][1];
			SET(S_RA_MINUS_X) = vectors[1][0];
			SET(S_RA_MINUS_Y) = vectors[1][1];
			SET(S_DEC_X) = vectors[2][0];
			SET(S_DEC_Y) = vectors[2][1];
			SET(S_DEC_MINUS_X) = vectors[3][0];
			SET(S_DEC_MINUS_Y) = vectors[3][1];
			double ra_plus = hypot(vectors[0][0], vectors[0][1]);
			double ra_minus = hypot(vectors[1][0], vectors[1][1]);
			double dec_plus = hypot(vectors[2][0], vectors[2][1]);
			double dec_minus = hypot(vectors[3][0], vectors[3][1]);
			indigo_update_property(device, AGENT_AO_SETTINGS_PROPERTY, NULL);
			save_config(device);
			indigo_send_message(device, OK_PROPERTY, "AO calibrated: %.3f / %.3f px/step along +RA / -RA, %.3f / %.3f px/step along +Dec / -Dec", ra_plus, ra_minus, dec_plus, dec_minus);
		}
	}
	stop_frames(device);
	ao_center(device);
	process_end(device, ok, ok ? "AO calibration finished" : "AO calibration failed");
}

// pulse and measurement of the star displacement it makes
static bool pulse_and_measure(indigo_device *device, double ra_seconds, double dec_seconds, double *vx, double *vy) {
	int frames = (int)SET(S_CAL_FRAMES);
	double bx, by, ax, ay;
	if (!measure_average(device, frames, &bx, &by) || !pulse_guide(device, ra_seconds, dec_seconds)) {
		return false;
	}
	indigo_usleep(1000000);
	if (!measure_average(device, frames, &ax, &ay)) {
		return false;
	}
	*vx = ax - bx;
	*vy = ay - by;
	return true;
}

// The mount is pulsed in all directions while the AO holds the centre, which gives the orientation and the speed of the OAG image in each direction
static void bump_calibrate_process(indigo_device *device) {
	process_begin(device, AGENT_AO_PHASE_BUMP_CALIBRATING);
	bool ok = true;
	if (!INDIGO_FILTER_GUIDER_SELECTED) {
		indigo_send_message(device, ALERT_PROPERTY, "No guider selected");
		ok = false;
	} else if (guider_agent_busy(device)) {
		indigo_send_message(device, ALERT_PROPERTY, "The Guider Agent is busy");
		ok = false;
	}
	ok = ok && ao_ready(device) && start_frames(device) && ao_center(device);
	double pulse = SET(S_CAL_PULSE);
	// north, south, west, east, each pulsed twice: the first pulse takes up the backlash of the reversal, the second one is measured
	static const double directions[4][2] = { { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 } };
	double responses[4][2] = { { 0 } };
	double ignored_x, ignored_y;
	for (int i = 0; ok && i < 4 && !aborted(device); i++) {
		ok = pulse_and_measure(device, pulse * directions[i][0], pulse * directions[i][1], &ignored_x, &ignored_y);
		ok = ok && pulse_and_measure(device, pulse * directions[i][0], pulse * directions[i][1], &responses[i][0], &responses[i][1]);
	}
	if (ok && !aborted(device)) {
		bool weak = false;
		for (int i = 0; i < 4; i++) {
			weak = weak || hypot(responses[i][0], responses[i][1]) / pulse < 0.01;
		}
		double cos_dec = cos(M_PI * DEVICE_PRIVATE_DATA->mount_dec / 180);
		if (cos_dec < 0.05) {
			cos_dec = 0.05;
		}
		// px/s of every direction, those of RA at the equator
		response_model model;
		for (int k = 0; k < 2; k++) {
			model.pos[1][k] = round(10000 * responses[0][k] / pulse) / 10000;
			model.neg[1][k] = round(10000 * responses[1][k] / pulse) / 10000;
			model.pos[0][k] = round(10000 * responses[2][k] / pulse / cos_dec) / 10000;
			model.neg[0][k] = round(10000 * responses[3][k] / pulse / cos_dec) / 10000;
		}
		if (weak) {
			indigo_send_message(device, ALERT_PROPERTY, "The star does not move with the guide pulses, check the guide port, the camera and the AO");
			ok = false;
		} else if (!model_valid(&model)) {
			indigo_send_message(device, ALERT_PROPERTY, "The OAG directions are not usable, the neighbouring ones must follow each other around the image %d to %d degrees apart", MODEL_MIN_GAP, MODEL_MAX_GAP);
			ok = false;
		} else {
			SET(S_WEST_X) = model.pos[0][0];
			SET(S_WEST_Y) = model.pos[0][1];
			SET(S_EAST_X) = model.neg[0][0];
			SET(S_EAST_Y) = model.neg[0][1];
			SET(S_NORTH_X) = model.pos[1][0];
			SET(S_NORTH_Y) = model.pos[1][1];
			SET(S_SOUTH_X) = model.neg[1][0];
			SET(S_SOUTH_Y) = model.neg[1][1];
			SET(S_OAG_SOP) = DEVICE_PRIVATE_DATA->mount_sop;
			indigo_update_property(device, AGENT_AO_SETTINGS_PROPERTY, NULL);
			save_config(device);
			indigo_send_message(device, OK_PROPERTY, "OAG calibrated: %.3f / %.3f px/s along +RA / -RA at the equator, %.3f / %.3f px/s along +Dec / -Dec", hypot(model.pos[0][0], model.pos[0][1]), hypot(model.neg[0][0], model.neg[0][1]), hypot(model.pos[1][0], model.pos[1][1]), hypot(model.neg[1][0], model.neg[1][1]));
		}
	}
	stop_frames(device);
	ao_center(device);
	process_end(device, ok, ok ? "OAG mount calibration finished" : "OAG mount calibration failed");
}

// The guider is about to move the main image on purpose, the loop must not cancel that
static bool dither_active(indigo_device *device) {
	return guider_agent_busy(device) && (DEVICE_PRIVATE_DATA->guider_dithering != 0 || DEVICE_PRIVATE_DATA->guider_dither_state == INDIGO_BUSY_STATE);
}

// An exposure of the related Imager Agent has finished: tilt the star by a new random offset, the AO moves at once and the
// reference of the loop follows, so that the loop does not undo the dither. The dither is the offset from the reference, not a sum of steps.
static void dither_step(indigo_device *device) {
	unsigned done = DEVICE_PRIVATE_DATA->imager_exposures;
	if (done == DEVICE_PRIVATE_DATA->dither_seen) {
		return;
	}
	DEVICE_PRIVATE_DATA->dither_count += done - DEVICE_PRIVATE_DATA->dither_seen;
	DEVICE_PRIVATE_DATA->dither_seen = done;
	double amplitude = SET(S_DITHER_AMPLITUDE);
	if (amplitude <= 0 || DEVICE_PRIVATE_DATA->dither_count < (unsigned)SET(S_DITHER_FRAMES) || DEVICE_PRIVATE_DATA->imager_start_state != INDIGO_BUSY_STATE || guider_agent_related(device)) {
		return;
	}
	DEVICE_PRIVATE_DATA->dither_count = 0;
	double x = amplitude * (2 * drand48() - 1);
	double y = amplitude * (2 * drand48() - 1);
	double steps_ra, steps_dec;
	px_to_steps(device, x - DEVICE_PRIVATE_DATA->dither_x, y - DEVICE_PRIVATE_DATA->dither_y, &steps_ra, &steps_dec);
	double new_ra = DEVICE_PRIVATE_DATA->want_ra + steps_ra;
	double new_dec = DEVICE_PRIVATE_DATA->want_dec + steps_dec;
	DEVICE_PRIVATE_DATA->saturated = hypot(new_ra, new_dec) > deflection_limit(device);
	ao_move_to(device, new_ra, new_dec);
	DEVICE_PRIVATE_DATA->dither_x = x;
	DEVICE_PRIVATE_DATA->dither_y = y;
	DEVICE_PRIVATE_DATA->skip_frames = DITHER_SKIP_FRAMES;
	update_stats(device);
}

// The fast loop: every frame the AO is moved to reduce the displacement of the star, the slow loop bumps the mount
static void tracking_process(indigo_device *device) {
	process_begin(device, AGENT_AO_PHASE_TRACKING);
	bool ok = ao_ready(device);
	if (ok && !ao_calibrated(device)) {
		indigo_send_message(device, ALERT_PROPERTY, "The AO is not calibrated");
		ok = false;
	}
	ok = ok && start_frames(device) && ao_center(device);
	bump_reset(device);
	DEVICE_PRIVATE_DATA->bump_active = false;
	DEVICE_PRIVATE_DATA->skip_frames = 0;
	DEVICE_PRIVATE_DATA->dither_seen = DEVICE_PRIVATE_DATA->imager_exposures;
	DEVICE_PRIVATE_DATA->dither_count = 0;
	bool frozen = false;
	double last_time = indigo_monotonic_time();
	DEVICE_PRIVATE_DATA->stats_time = last_time;
	DEVICE_PRIVATE_DATA->rate_frames = 0;
	while (ok && !aborted(device)) {
		if (DEVICE_PRIVATE_DATA->mount_slewing || dither_active(device)) {
			if (!frozen) {
				frozen = true;
				ao_center(device);
				if (DEVICE_PRIVATE_DATA->mount_slewing) {
					bump_reset(device);
				}
				indigo_send_message(device, BUSY_PROPERTY, DEVICE_PRIVATE_DATA->mount_slewing ? "Mount is slewing, the AO loop is paused" : "Dithering, the AO loop is paused");
				update_stats(device);
			}
			indigo_usleep(100000);
			continue;
		}
		if (frozen) {
			frozen = false;
			relock(device);
			last_time = indigo_monotonic_time();
		}
		dither_step(device);
		if (!get_frame(device, FRAME_TIMEOUT)) {
			if (aborted(device)) {
				break;
			}
			DEVICE_PRIVATE_DATA->lost_frames++;
		} else if (DEVICE_PRIVATE_DATA->skip_frames > 0) {
			// frames exposed before the AO settled show the star at its old position
			DEVICE_PRIVATE_DATA->skip_frames--;
		} else {
			double dx, dy;
			if (analyze_frame(device, &dx, &dy)) {
				dx -= DEVICE_PRIVATE_DATA->dither_x;
				dy -= DEVICE_PRIVATE_DATA->dither_y;
				DEVICE_PRIVATE_DATA->lost_frames = 0;
				DEVICE_PRIVATE_DATA->rate_frames++;
				double min_move = SET(S_MIN_MOVE);
				if (hypot(dx, dy) >= min_move) {
					double steps_ra, steps_dec;
					px_to_steps(device, -SET(S_GAIN) * dx, -SET(S_GAIN) * dy, &steps_ra, &steps_dec);
					double new_ra = DEVICE_PRIVATE_DATA->want_ra + steps_ra;
					double new_dec = DEVICE_PRIVATE_DATA->want_dec + steps_dec;
					DEVICE_PRIVATE_DATA->saturated = hypot(new_ra, new_dec) > deflection_limit(device);
					ao_move_to(device, new_ra, new_dec);
				}
				DEVICE_PRIVATE_DATA->rmse_x_sum += dx * dx;
				DEVICE_PRIVATE_DATA->rmse_y_sum += dy * dy;
				DEVICE_PRIVATE_DATA->rmse_count++;
				STAT(T_DRIFT_X) = round(1000 * dx) / 1000;
				STAT(T_DRIFT_Y) = round(1000 * dy) / 1000;
				STAT(T_SNR) = round(10 * DEVICE_PRIVATE_DATA->snr) / 10;
			} else {
				DEVICE_PRIVATE_DATA->lost_frames++;
			}
		}
		double now = indigo_monotonic_time();
		if (DEVICE_PRIVATE_DATA->lost_frames > (int)SET(S_LOST_FRAMES)) {
			indigo_send_message(device, ALERT_PROPERTY, "The star is lost, the AO is centered and the loop looks for a star again");
			ao_center(device);
			relock(device);
			DEVICE_PRIVATE_DATA->lost_frames = 0;
		}
		bump_step(device, now - last_time);
		last_time = now;
		if (now - DEVICE_PRIVATE_DATA->stats_time >= STATS_PERIOD) {
			double a_x, a_y;
			steps_to_px(device, DEVICE_PRIVATE_DATA->pos_ra, DEVICE_PRIVATE_DATA->pos_dec, &a_x, &a_y);
			STAT(T_FRAME_RATE) = round(10 * DEVICE_PRIVATE_DATA->rate_frames / (now - DEVICE_PRIVATE_DATA->stats_time)) / 10;
			if (DEVICE_PRIVATE_DATA->rmse_count > 0) {
				STAT(T_RMSE_X) = round(1000 * sqrt(DEVICE_PRIVATE_DATA->rmse_x_sum / DEVICE_PRIVATE_DATA->rmse_count)) / 1000;
				STAT(T_RMSE_Y) = round(1000 * sqrt(DEVICE_PRIVATE_DATA->rmse_y_sum / DEVICE_PRIVATE_DATA->rmse_count)) / 1000;
			}
			STAT(T_OFFSET_X) = round(1000 * a_x) / 1000;
			STAT(T_OFFSET_Y) = round(1000 * a_y) / 1000;
			update_stats(device);
			DEVICE_PRIVATE_DATA->stats_time = now;
			DEVICE_PRIVATE_DATA->rate_frames = 0;
		}
	}
	DEVICE_PRIVATE_DATA->bump_active = false;
	stop_frames(device);
	if (INDIGO_FILTER_AO_SELECTED) {
		ao_center(device);
	}
	bump_reset(device);
	process_end(device, ok, ok ? "AO loop stopped" : "AO loop failed");
}

// -------------------------------------------------------------------------------- snooping

static void snoop_guider_agent(indigo_client *client, indigo_property *property) {
	indigo_device *device = FILTER_CLIENT_CONTEXT->device;
	char *related = indigo_filter_first_related_agent(device, GUIDER_AGENT_PREFIX);
	if (related == NULL || strcmp(related, property->device)) {
		return;
	}
	if (!strcmp(property->name, AGENT_GUIDER_STATS_PROPERTY_NAME)) {
		for (int i = 0; i < property->count; i++) {
			indigo_item *item = property->items + i;
			if (!strcmp(item->name, AGENT_GUIDER_STATS_DITHERING_ITEM_NAME)) {
				CLIENT_PRIVATE_DATA->guider_dithering = item->number.value;
			}
		}
	} else if (!strcmp(property->name, AGENT_START_PROCESS_PROPERTY_NAME)) {
		CLIENT_PRIVATE_DATA->guider_start_state = property->state;
	} else if (!strcmp(property->name, AGENT_GUIDER_DITHER_PROPERTY_NAME)) {
		CLIENT_PRIVATE_DATA->guider_dither_state = property->state;
	}
}

// The end of an exposure of the related Imager Agent is CCD_EXPOSURE going from BUSY to OK
static void snoop_imager_agent(indigo_client *client, indigo_property *property) {
	indigo_device *device = FILTER_CLIENT_CONTEXT->device;
	char *related = indigo_filter_first_related_agent(device, IMAGER_AGENT_PREFIX);
	if (related == NULL || strcmp(related, property->device)) {
		return;
	}
	if (!strcmp(property->name, CCD_EXPOSURE_PROPERTY_NAME)) {
		if (CLIENT_PRIVATE_DATA->imager_exposure_state == INDIGO_BUSY_STATE && property->state == INDIGO_OK_STATE) {
			CLIENT_PRIVATE_DATA->imager_exposures++;
		}
		CLIENT_PRIVATE_DATA->imager_exposure_state = property->state;
	} else if (!strcmp(property->name, AGENT_START_PROCESS_PROPERTY_NAME)) {
		CLIENT_PRIVATE_DATA->imager_start_state = property->state;
	}
}

// the properties of the selected devices come from the agent itself, with the names it publishes
static void snoop_changes(indigo_client *client, indigo_device *device, indigo_property *property) {
	if (!strcmp(property->name, AO_GUIDE_RA_PROPERTY_NAME)) {
		CLIENT_PRIVATE_DATA->ao_ra_state = property->state;
		CLIENT_PRIVATE_DATA->ao_ra_serial++;
	} else if (!strcmp(property->name, AO_GUIDE_DEC_PROPERTY_NAME)) {
		CLIENT_PRIVATE_DATA->ao_dec_state = property->state;
		CLIENT_PRIVATE_DATA->ao_dec_serial++;
	} else if (!strcmp(property->name, GUIDER_GUIDE_RA_PROPERTY_NAME)) {
		CLIENT_PRIVATE_DATA->guide_ra_state = property->state;
		CLIENT_PRIVATE_DATA->guide_ra_serial++;
	} else if (!strcmp(property->name, GUIDER_GUIDE_DEC_PROPERTY_NAME)) {
		CLIENT_PRIVATE_DATA->guide_dec_state = property->state;
		CLIENT_PRIVATE_DATA->guide_dec_serial++;
	} else if (!strcmp(property->name, CCD_STREAMING_PROPERTY_NAME)) {
		CLIENT_PRIVATE_DATA->has_streaming = true;
		CLIENT_PRIVATE_DATA->streaming_state = property->state;
	} else if (!strcmp(property->name, CCD_EXPOSURE_PROPERTY_NAME)) {
		CLIENT_PRIVATE_DATA->exposure_state = property->state;
	} else if (!strcmp(property->name, MOUNT_EQUATORIAL_COORDINATES_PROPERTY_NAME)) {
		CLIENT_PRIVATE_DATA->mount_slewing = property->state == INDIGO_BUSY_STATE;
		for (int i = 0; i < property->count; i++) {
			if (!strcmp(property->items[i].name, MOUNT_EQUATORIAL_COORDINATES_DEC_ITEM_NAME)) {
				CLIENT_PRIVATE_DATA->mount_dec = property->items[i].number.value;
			}
		}
	} else if (!strcmp(property->name, MOUNT_SIDE_OF_PIER_PROPERTY_NAME)) {
		for (int i = 0; i < property->count; i++) {
			indigo_item *item = property->items + i;
			if (!strcmp(item->name, MOUNT_SIDE_OF_PIER_WEST_ITEM_NAME) && item->sw.value) {
				CLIENT_PRIVATE_DATA->mount_sop = 1;
			} else if (!strcmp(item->name, MOUNT_SIDE_OF_PIER_EAST_ITEM_NAME) && item->sw.value) {
				CLIENT_PRIVATE_DATA->mount_sop = -1;
			}
		}
	}
}

// the driver name of the AO tells how it is positioned: the SBIG AO is absolute, the others relative
static void snoop_info(indigo_client *client, indigo_property *property) {
	indigo_device *device = FILTER_CLIENT_CONTEXT->device;
	for (int i = 0; i < property->count; i++) {
		indigo_item *item = property->items + i;
		if (!strcmp(item->name, INFO_DEVICE_DRIVER_ITEM_NAME) && strstr(item->text.value, "sbig") != NULL) {
			if (!AGENT_AO_MODE_ABSOLUTE_ITEM->sw.value && !FILTER_CLIENT_CONTEXT->running_process) {
				indigo_set_switch(AGENT_AO_MODE_PROPERTY, AGENT_AO_MODE_ABSOLUTE_ITEM, true);
				SET(S_RANGE) = 100;
				SET(S_MAX_STEP) = 100;
				indigo_update_property(device, AGENT_AO_MODE_PROPERTY, "SBIG AO detected, absolute positioning");
				indigo_update_property(device, AGENT_AO_SETTINGS_PROPERTY, NULL);
			}
		}
	}
}

// -------------------------------------------------------------------------------- INDIGO agent device implementation

static bool validate_related_agent(indigo_device *device, indigo_property *info_property, int mask) {
	return !strncmp(info_property->device, GUIDER_AGENT_PREFIX, strlen(GUIDER_AGENT_PREFIX)) || !strncmp(info_property->device, IMAGER_AGENT_PREFIX, strlen(IMAGER_AGENT_PREFIX));
}

static indigo_result agent_enumerate_properties(indigo_device *device, indigo_client *client, indigo_property *property);

static indigo_property *init_numbers(indigo_device *device, const char *name, const char *label, indigo_property_perm perm, const setting_definition *definitions, int count) {
	indigo_property *result = indigo_init_number_property(NULL, device->name, name, "Agent", label, INDIGO_OK_STATE, perm, count);
	if (result == NULL) {
		return NULL;
	}
	for (int i = 0; i < count; i++) {
		indigo_init_number_item(result->items + i, definitions[i].name, definitions[i].label, definitions[i].min, definitions[i].max, definitions[i].step, definitions[i].value);
	}
	return result;
}

static indigo_result agent_device_attach(indigo_device *device) {
	assert(device != NULL);
	assert(DEVICE_PRIVATE_DATA != NULL);
	if (indigo_filter_device_attach(device, DRIVER_NAME, DRIVER_VERSION, INDIGO_INTERFACE_AO | INDIGO_INTERFACE_CCD | INDIGO_INTERFACE_GUIDER | INDIGO_INTERFACE_MOUNT) == INDIGO_OK) {
		// -------------------------------------------------------------------------------- Device properties
		FILTER_AO_LIST_PROPERTY->hidden = false;
		FILTER_CCD_LIST_PROPERTY->hidden = false;
		FILTER_GUIDER_LIST_PROPERTY->hidden = false;
		FILTER_MOUNT_LIST_PROPERTY->hidden = false;
		FILTER_DEVICE_CONTEXT->validate_related_agent = validate_related_agent;
		FILTER_RELATED_AGENT_LIST_PROPERTY->hidden = false;
		// -------------------------------------------------------------------------------- Process control
		AGENT_START_PROCESS_PROPERTY = indigo_init_switch_property(NULL, device->name, AGENT_START_PROCESS_PROPERTY_NAME, "Agent", "Start process", INDIGO_OK_STATE, INDIGO_RW_PERM, INDIGO_ANY_OF_MANY_RULE, 7);
		if (AGENT_START_PROCESS_PROPERTY == NULL) {
			return INDIGO_FAILED;
		}
		indigo_init_switch_item(AGENT_AO_CALIBRATION_ITEM, AGENT_AO_CALIBRATION_ITEM_NAME, "Calibrate AO", false);
		indigo_init_switch_item(AGENT_AO_BUMP_CALIBRATION_ITEM, AGENT_AO_BUMP_CALIBRATION_ITEM_NAME, "Calibrate OAG on the mount", false);
		indigo_init_switch_item(AGENT_AO_EXERCISE_ITEM, AGENT_AO_EXERCISE_ITEM_NAME, "Exercise AO", false);
		indigo_init_switch_item(AGENT_AO_CENTER_ITEM, AGENT_AO_CENTER_ITEM_NAME, "Center AO", false);
		indigo_init_switch_item(AGENT_AO_TRACKING_ITEM, AGENT_AO_TRACKING_ITEM_NAME, "Start AO loop", false);
		indigo_init_switch_item(AGENT_AO_DARK_FRAME_ITEM, AGENT_AO_DARK_FRAME_ITEM_NAME, "Take dark frame", false);
		indigo_init_switch_item(AGENT_AO_DARK_RESET_ITEM, AGENT_AO_DARK_RESET_ITEM_NAME, "Reset dark frame", false);
		AGENT_ABORT_PROCESS_PROPERTY = indigo_init_switch_property(NULL, device->name, AGENT_ABORT_PROCESS_PROPERTY_NAME, "Agent", "Abort process", INDIGO_OK_STATE, INDIGO_RW_PERM, INDIGO_AT_MOST_ONE_RULE, 1);
		if (AGENT_ABORT_PROCESS_PROPERTY == NULL) {
			return INDIGO_FAILED;
		}
		indigo_init_switch_item(AGENT_ABORT_PROCESS_ITEM, AGENT_ABORT_PROCESS_ITEM_NAME, "Abort", false);
		// -------------------------------------------------------------------------------- Modes
		AGENT_AO_MODE_PROPERTY = indigo_init_switch_property(NULL, device->name, AGENT_AO_MODE_PROPERTY_NAME, "Agent", "AO positioning", INDIGO_OK_STATE, INDIGO_RW_PERM, INDIGO_ONE_OF_MANY_RULE, 2);
		if (AGENT_AO_MODE_PROPERTY == NULL) {
			return INDIGO_FAILED;
		}
		indigo_init_switch_item(AGENT_AO_MODE_RELATIVE_ITEM, AGENT_AO_MODE_RELATIVE_ITEM_NAME, "Relative steps", true);
		indigo_init_switch_item(AGENT_AO_MODE_ABSOLUTE_ITEM, AGENT_AO_MODE_ABSOLUTE_ITEM_NAME, "Absolute percent", false);
		AGENT_AO_BUMP_MODE_PROPERTY = indigo_init_switch_property(NULL, device->name, AGENT_AO_BUMP_MODE_PROPERTY_NAME, "Agent", "Mount bump", INDIGO_OK_STATE, INDIGO_RW_PERM, INDIGO_ONE_OF_MANY_RULE, 2);
		if (AGENT_AO_BUMP_MODE_PROPERTY == NULL) {
			return INDIGO_FAILED;
		}
		indigo_init_switch_item(AGENT_AO_BUMP_MODE_DIRECT_ITEM, AGENT_AO_BUMP_MODE_DIRECT_ITEM_NAME, "Direct guide pulses", true);
		indigo_init_switch_item(AGENT_AO_BUMP_MODE_OFF_ITEM, AGENT_AO_BUMP_MODE_OFF_ITEM_NAME, "Off", false);
		AGENT_AO_DEC_MODE_PROPERTY = indigo_init_switch_property(NULL, device->name, AGENT_AO_DEC_MODE_PROPERTY_NAME, "Agent", "Dec bump mode", INDIGO_OK_STATE, INDIGO_RW_PERM, INDIGO_ONE_OF_MANY_RULE, 3);
		if (AGENT_AO_DEC_MODE_PROPERTY == NULL) {
			return INDIGO_FAILED;
		}
		indigo_init_switch_item(AGENT_AO_DEC_MODE_BOTH_ITEM, AGENT_AO_DEC_MODE_BOTH_ITEM_NAME, "North and south", true);
		indigo_init_switch_item(AGENT_AO_DEC_MODE_NORTH_ITEM, AGENT_AO_DEC_MODE_NORTH_ITEM_NAME, "North only", false);
		indigo_init_switch_item(AGENT_AO_DEC_MODE_SOUTH_ITEM, AGENT_AO_DEC_MODE_SOUTH_ITEM_NAME, "South only", false);
		// -------------------------------------------------------------------------------- Settings and statistics
		AGENT_AO_SETTINGS_PROPERTY = init_numbers(device, AGENT_AO_SETTINGS_PROPERTY_NAME, "Settings", INDIGO_RW_PERM, settings_definitions, S_COUNT);
		AGENT_AO_STATS_PROPERTY = init_numbers(device, AGENT_AO_STATS_PROPERTY_NAME, "Statistics", INDIGO_RO_PERM, stats_definitions, T_COUNT);
		if (AGENT_AO_SETTINGS_PROPERTY == NULL || AGENT_AO_STATS_PROPERTY == NULL) {
			return INDIGO_FAILED;
		}
		AGENT_AO_PREVIEW_PROPERTY = indigo_init_blob_property(NULL, device->name, AGENT_AO_PREVIEW_PROPERTY_NAME, "Agent", "Preview of the image the stars are measured on", INDIGO_OK_STATE, 1);
		if (AGENT_AO_PREVIEW_PROPERTY == NULL) {
			return INDIGO_FAILED;
		}
		indigo_init_blob_item(AGENT_AO_PREVIEW_ITEM, AGENT_AO_PREVIEW_ITEM_NAME, "Image data");
		// --------------------------------------------------------------------------------
		pthread_mutex_init(&DEVICE_PRIVATE_DATA->mutex, NULL);
		pthread_mutex_init(&DEVICE_PRIVATE_DATA->frame_mutex, NULL);
		CONFIG_PROPERTY->hidden = true;
		PROFILE_PROPERTY->hidden = true;
		CONNECTION_PROPERTY->hidden = true;
		indigo_load_properties(device, false);
		INDIGO_DEVICE_ATTACH_LOG(DRIVER_NAME, device->name);
		return agent_enumerate_properties(device, NULL, NULL);
	}
	return INDIGO_FAILED;
}

static indigo_result agent_enumerate_properties(indigo_device *device, indigo_client *client, indigo_property *property) {
	if (client != NULL && client == FILTER_DEVICE_CONTEXT->client) {
		return INDIGO_OK;
	}
	INDIGO_DEFINE_MATCHING_PROPERTY(AGENT_START_PROCESS_PROPERTY);
	INDIGO_DEFINE_MATCHING_PROPERTY(AGENT_ABORT_PROCESS_PROPERTY);
	INDIGO_DEFINE_MATCHING_PROPERTY(AGENT_AO_MODE_PROPERTY);
	INDIGO_DEFINE_MATCHING_PROPERTY(AGENT_AO_BUMP_MODE_PROPERTY);
	INDIGO_DEFINE_MATCHING_PROPERTY(AGENT_AO_DEC_MODE_PROPERTY);
	INDIGO_DEFINE_MATCHING_PROPERTY(AGENT_AO_SETTINGS_PROPERTY);
	INDIGO_DEFINE_MATCHING_PROPERTY(AGENT_AO_STATS_PROPERTY);
	INDIGO_DEFINE_MATCHING_PROPERTY(AGENT_AO_PREVIEW_PROPERTY);
	return indigo_filter_enumerate_properties(device, client, property);
}

static void stop_process(indigo_device *device) {
	AGENT_ABORT_PROCESS_PROPERTY->state = INDIGO_BUSY_STATE;
	indigo_cancel_pending_handlers(device);
	for (int i = 0; i < 5000 && FILTER_DEVICE_CONTEXT->running_process; i++) {
		indigo_usleep(1000);
	}
	indigo_cancel_all_timers(device);
}

static void start_process(indigo_device *device, void (*process)(indigo_device *device)) {
	AGENT_START_PROCESS_PROPERTY->state = INDIGO_BUSY_STATE;
	indigo_update_property(device, AGENT_START_PROCESS_PROPERTY, NULL);
	indigo_set_timer(device, 0, process, NULL);
}

static indigo_result agent_change_property(indigo_device *device, indigo_client *client, indigo_property *property) {
	assert(device != NULL);
	assert(DEVICE_CONTEXT != NULL);
	assert(property != NULL);
	if (indigo_property_match_changeable(AGENT_START_PROCESS_PROPERTY, property)) {
// -------------------------------------------------------------------------------- AGENT_START_PROCESS
		if (AGENT_START_PROCESS_PROPERTY->state == INDIGO_BUSY_STATE || FILTER_DEVICE_CONTEXT->running_process) {
			indigo_send_message(device, BUSY_PROPERTY, "A process is running, abort it first");
			return INDIGO_OK;
		}
		indigo_property_copy_values(AGENT_START_PROCESS_PROPERTY, property, false);
		if (AGENT_AO_CALIBRATION_ITEM->sw.value) {
			start_process(device, calibrate_process);
		} else if (AGENT_AO_BUMP_CALIBRATION_ITEM->sw.value) {
			start_process(device, bump_calibrate_process);
		} else if (AGENT_AO_EXERCISE_ITEM->sw.value) {
			start_process(device, exercise_process);
		} else if (AGENT_AO_CENTER_ITEM->sw.value) {
			start_process(device, center_process);
		} else if (AGENT_AO_TRACKING_ITEM->sw.value) {
			start_process(device, tracking_process);
		} else if (AGENT_AO_DARK_FRAME_ITEM->sw.value) {
			start_process(device, dark_process);
		} else if (AGENT_AO_DARK_RESET_ITEM->sw.value) {
			start_process(device, dark_reset_process);
		} else {
			AGENT_START_PROCESS_PROPERTY->state = INDIGO_OK_STATE;
			indigo_update_property(device, AGENT_START_PROCESS_PROPERTY, NULL);
		}
		return INDIGO_OK;
	} else if (indigo_property_match_changeable(AGENT_ABORT_PROCESS_PROPERTY, property)) {
// -------------------------------------------------------------------------------- AGENT_ABORT_PROCESS
		indigo_property_copy_values(AGENT_ABORT_PROCESS_PROPERTY, property, false);
		if (AGENT_ABORT_PROCESS_ITEM->sw.value) {
			AGENT_ABORT_PROCESS_ITEM->sw.value = false;
			if (FILTER_DEVICE_CONTEXT->running_process) {
				AGENT_ABORT_PROCESS_PROPERTY->state = INDIGO_BUSY_STATE;
			} else {
				AGENT_ABORT_PROCESS_PROPERTY->state = INDIGO_OK_STATE;
			}
		}
		indigo_update_property(device, AGENT_ABORT_PROCESS_PROPERTY, NULL);
		return INDIGO_OK;
	} else if (indigo_property_match_changeable(AGENT_AO_MODE_PROPERTY, property)) {
// -------------------------------------------------------------------------------- AGENT_AO_MODE
		if (FILTER_DEVICE_CONTEXT->running_process) {
			AGENT_AO_MODE_PROPERTY->state = INDIGO_ALERT_STATE;
			indigo_update_property(device, AGENT_AO_MODE_PROPERTY, "The positioning cannot change while a process is running");
			return INDIGO_OK;
		}
		indigo_property_copy_values(AGENT_AO_MODE_PROPERTY, property, false);
		AGENT_AO_MODE_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, AGENT_AO_MODE_PROPERTY, NULL);
		save_config(device);
		return INDIGO_OK;
	} else if (indigo_property_match_changeable(AGENT_AO_BUMP_MODE_PROPERTY, property)) {
// -------------------------------------------------------------------------------- AGENT_AO_BUMP_MODE
		indigo_property_copy_values(AGENT_AO_BUMP_MODE_PROPERTY, property, false);
		AGENT_AO_BUMP_MODE_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, AGENT_AO_BUMP_MODE_PROPERTY, NULL);
		save_config(device);
		return INDIGO_OK;
	} else if (indigo_property_match_changeable(AGENT_AO_DEC_MODE_PROPERTY, property)) {
// -------------------------------------------------------------------------------- AGENT_AO_DEC_MODE
		indigo_property_copy_values(AGENT_AO_DEC_MODE_PROPERTY, property, false);
		AGENT_AO_DEC_MODE_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, AGENT_AO_DEC_MODE_PROPERTY, NULL);
		save_config(device);
		return INDIGO_OK;
	} else if (indigo_property_match_changeable(AGENT_AO_SETTINGS_PROPERTY, property)) {
// -------------------------------------------------------------------------------- AGENT_AO_SETTINGS
		indigo_property_copy_values(AGENT_AO_SETTINGS_PROPERTY, property, false);
		AGENT_AO_SETTINGS_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, AGENT_AO_SETTINGS_PROPERTY, NULL);
		save_config(device);
		return INDIGO_OK;
	}
	return indigo_filter_change_property(device, client, property);
}

static indigo_result agent_device_detach(indigo_device *device) {
	assert(device != NULL);
	stop_process(device);
	save_config(device);
	indigo_release_property(AGENT_START_PROCESS_PROPERTY);
	indigo_release_property(AGENT_ABORT_PROCESS_PROPERTY);
	indigo_release_property(AGENT_AO_MODE_PROPERTY);
	indigo_release_property(AGENT_AO_BUMP_MODE_PROPERTY);
	indigo_release_property(AGENT_AO_DEC_MODE_PROPERTY);
	indigo_release_property(AGENT_AO_SETTINGS_PROPERTY);
	indigo_release_property(AGENT_AO_STATS_PROPERTY);
	indigo_release_property(AGENT_AO_PREVIEW_PROPERTY);
	indigo_delete_frame_digest(&DEVICE_PRIVATE_DATA->reference);
	pthread_mutex_destroy(&DEVICE_PRIVATE_DATA->mutex);
	pthread_mutex_destroy(&DEVICE_PRIVATE_DATA->frame_mutex);
	indigo_safe_free(DEVICE_PRIVATE_DATA->frame_data);
	indigo_safe_free(DEVICE_PRIVATE_DATA->work_data);
	indigo_safe_free(DEVICE_PRIVATE_DATA->dark_data);
	indigo_safe_free(DEVICE_PRIVATE_DATA->preview_data);
	INDIGO_DEVICE_DETACH_LOG(DRIVER_NAME, device->name);
	return indigo_filter_device_detach(device);
}

// -------------------------------------------------------------------------------- INDIGO agent client implementation

// The image of the camera is copied at once, the processing is up to the worker
static void snoop_image(indigo_client *client, indigo_property *property) {
	if (property->state != INDIGO_OK_STATE || property->count < 1) {
		return;
	}
	indigo_item *item = property->items;
	pthread_mutex_lock(&CLIENT_PRIVATE_DATA->frame_mutex);
	if (CLIENT_PRIVATE_DATA->frames_wanted) {
		if (item->blob.value && item->blob.size > 0) {
			if (item->blob.size > CLIENT_PRIVATE_DATA->frame_capacity) {
				CLIENT_PRIVATE_DATA->frame_data = indigo_safe_realloc(CLIENT_PRIVATE_DATA->frame_data, item->blob.size);
				CLIENT_PRIVATE_DATA->frame_capacity = item->blob.size;
			}
			memcpy(CLIENT_PRIVATE_DATA->frame_data, item->blob.value, item->blob.size);
			CLIENT_PRIVATE_DATA->frame_size = item->blob.size;
			CLIENT_PRIVATE_DATA->frame_url[0] = 0;
		} else {
			INDIGO_COPY_VALUE(CLIENT_PRIVATE_DATA->frame_url, item->blob.url);
		}
		CLIENT_PRIVATE_DATA->frame_serial++;
	}
	pthread_mutex_unlock(&CLIENT_PRIVATE_DATA->frame_mutex);
}

static indigo_result agent_define_property(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	if (device == FILTER_CLIENT_CONTEXT->device) {
		snoop_changes(client, device, property);
	} else if (!strcmp(property->name, INFO_PROPERTY_NAME)) {
		if (!strcmp(property->device, FILTER_CLIENT_CONTEXT->device_name[INDIGO_FILTER_AO_INDEX])) {
			snoop_info(client, property);
		}
	} else {
		snoop_guider_agent(client, property);
		snoop_imager_agent(client, property);
	}
	return indigo_filter_define_property(client, device, property, message);
}

static indigo_result agent_update_property(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	if (device == FILTER_CLIENT_CONTEXT->device) {
		if (!strcmp(property->name, CCD_IMAGE_PROPERTY_NAME)) {
			snoop_image(client, property);
		} else {
			snoop_changes(client, device, property);
		}
	} else {
		snoop_guider_agent(client, property);
		snoop_imager_agent(client, property);
	}
	return indigo_filter_update_property(client, device, property, message);
}

static indigo_result agent_delete_property(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	if (device == FILTER_CLIENT_CONTEXT->device) {
		if (!strcmp(property->name, CCD_STREAMING_PROPERTY_NAME) || *property->name == 0) {
			CLIENT_PRIVATE_DATA->has_streaming = false;
		}
	}
	return indigo_filter_delete_property(client, device, property, message);
}

// -------------------------------------------------------------------------------- Initialization

static ao_agent_private_data *private_data = NULL;

static indigo_device *agent_device = NULL;
static indigo_client *agent_client = NULL;

indigo_result indigo_agent_ao(indigo_driver_action action, indigo_driver_info *info) {
	static indigo_device agent_device_template = INDIGO_DEVICE_INITIALIZER(
		AO_AGENT_NAME,
		agent_device_attach,
		agent_enumerate_properties,
		agent_change_property,
		NULL,
		agent_device_detach
	);

	static indigo_client agent_client_template = {
		AO_AGENT_NAME, false, NULL, INDIGO_OK, INDIGO_VERSION_CURRENT, NULL,
		indigo_filter_client_attach,
		agent_define_property,
		agent_update_property,
		agent_delete_property,
		NULL,
		indigo_filter_client_detach
	};

	static indigo_driver_action last_action = INDIGO_DRIVER_SHUTDOWN;

	SET_DRIVER_INFO(info, AO_AGENT_NAME, __FUNCTION__, DRIVER_VERSION, false, last_action);

	if (action == last_action) {
		return INDIGO_OK;
	}

	switch(action) {
		case INDIGO_DRIVER_INIT:
			last_action = action;
			private_data = indigo_safe_malloc(sizeof(ao_agent_private_data));
			agent_device = indigo_safe_malloc_copy(sizeof(indigo_device), &agent_device_template);
			agent_device->private_data = private_data;
			indigo_attach_device(agent_device);
			agent_client = indigo_safe_malloc_copy(sizeof(indigo_client), &agent_client_template);
			agent_client->client_context = agent_device->device_context;
			indigo_attach_client(agent_client);
			break;

		case INDIGO_DRIVER_SHUTDOWN:
			last_action = action;
			if (agent_device != NULL) {
				stop_process(agent_device);
			}
			if (agent_client != NULL) {
				indigo_detach_client(agent_client);
				free(agent_client);
				agent_client = NULL;
			}
			if (agent_device != NULL) {
				indigo_detach_device(agent_device);
				free(agent_device);
				agent_device = NULL;
			}
			if (private_data != NULL) {
				free(private_data);
				private_data = NULL;
			}
			break;

		case INDIGO_DRIVER_INFO:
			break;
	}
	return INDIGO_OK;
}
