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

// The AO Agent with the CCD simulator: the off-axis guide camera (CCD OAG Simulator) behind the simulated AO.
// Covered: the exercise visits the corners and returns to the centre, the AO calibration recovers the axes of the
// simulated AO (angle and scale), the closed loop cancels the slow tip-tilt of the OAG, and the processes refuse
// to start when the AO is not calibrated or not selected. Every case runs in a forked child with its own bus and
// a private HOME.

#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

#include <indigo/indigo_driver.h>
#include <indigo_drivers/agent_ao/indigo_agent_ao.h>
#include <indigo_drivers/ccd_simulator/indigo_ccd_simulator.h>

#include "../test_runner.h"

#define AO AO_AGENT_NAME
#define OAG_CAMERA "CCD OAG Simulator"
#define SETUP_CAMERA "CCD Guider Simulator"
#define AO_DEVICE "CCD Guider Simulator (AO)"
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define REQUIRE(c) do { if (!(c)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); indigo_test_failures++; return false; } } while (0)
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); indigo_test_failures++; dump_messages(); return; } } while (0)
#define WAIT_UNTIL(condition, timeout) ({ double deadline_ = indigo_monotonic_time() + (timeout); bool ready_ = (condition); while (!ready_ && indigo_monotonic_time() < deadline_) { indigo_usleep(5000); ready_ = (condition); } ready_; })

typedef struct {
	char device[INDIGO_NAME_SIZE];
	char name[INDIGO_NAME_SIZE];
	indigo_property *property;
	unsigned revision;
	unsigned states[4];
} observation;

static observation cache[4096];
static pthread_mutex_t cache_mutex = PTHREAD_MUTEX_INITIALIZER;
static indigo_client observer;
static char messages[262144];
static bool bus_started, observer_attached, ccd_started, ao_started;

static observation *find_observation(const char *device, const char *name) {
	observation *free_entry = NULL;
	for (unsigned i = 0; i < ARRAY_SIZE(cache); i++) {
		if (!*cache[i].name) {
			if (!free_entry) {
				free_entry = cache + i;
			}
		} else if (!strcmp(cache[i].device, device) && !strcmp(cache[i].name, name)) {
			return cache + i;
		}
	}
	if (free_entry) {
		INDIGO_COPY_NAME(free_entry->device, device);
		INDIGO_COPY_NAME(free_entry->name, name);
	}
	return free_entry;
}

static indigo_result observe(indigo_property *property, bool update) {
	pthread_mutex_lock(&cache_mutex);
	observation *entry = find_observation(property->device, property->name);
	if (entry && property->type != INDIGO_BLOB_VECTOR) {
		indigo_release_property(entry->property);
		entry->property = indigo_copy_property(NULL, property);
		entry->revision++;
		if (update && property->state <= INDIGO_ALERT_STATE) {
			entry->states[property->state]++;
		}
	}
	pthread_mutex_unlock(&cache_mutex);
	return INDIGO_OK;
}

static indigo_result defined(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	return observe(property, false);
}

static indigo_result updated(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	return observe(property, true);
}

static indigo_result deleted(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	pthread_mutex_lock(&cache_mutex);
	for (unsigned i = 0; i < ARRAY_SIZE(cache); i++) {
		observation *entry = cache + i;
		if (*entry->name && !strcmp(entry->device, property->device) && (!*property->name || !strcmp(entry->name, property->name))) {
			indigo_release_property(entry->property);
			memset(entry, 0, sizeof(*entry));
		}
	}
	pthread_mutex_unlock(&cache_mutex);
	return INDIGO_OK;
}

static indigo_result message_received(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	pthread_mutex_lock(&cache_mutex);
	if (message && strlen(messages) + strlen(device->name) + strlen(message) + 4 < sizeof(messages)) {
		strcat(messages, device->name);
		strcat(messages, ": ");
		strcat(messages, message);
		strcat(messages, "\n");
	}
	pthread_mutex_unlock(&cache_mutex);
	return INDIGO_OK;
}

static bool has_message(const char *text) {
	pthread_mutex_lock(&cache_mutex);
	bool result = strstr(messages, text) != NULL;
	pthread_mutex_unlock(&cache_mutex);
	return result;
}

static void dump_messages(void) {
	pthread_mutex_lock(&cache_mutex);
	fprintf(stderr, "---- agent messages ----\n%s------------------------\n", messages);
	pthread_mutex_unlock(&cache_mutex);
}

static unsigned revision(const char *device, const char *name) {
	pthread_mutex_lock(&cache_mutex);
	observation *entry = find_observation(device, name);
	unsigned result = entry ? entry->revision : 0;
	pthread_mutex_unlock(&cache_mutex);
	return result;
}

static unsigned state_count(const char *device, const char *name, indigo_property_state state) {
	pthread_mutex_lock(&cache_mutex);
	observation *entry = find_observation(device, name);
	unsigned result = entry ? entry->states[state] : 0;
	pthread_mutex_unlock(&cache_mutex);
	return result;
}

static int state(const char *device, const char *name) {
	pthread_mutex_lock(&cache_mutex);
	observation *entry = find_observation(device, name);
	int result = entry && entry->property ? (int)entry->property->state : -1;
	pthread_mutex_unlock(&cache_mutex);
	return result;
}

static bool item_lookup(const char *device, const char *name, const char *item_name, indigo_item *result) {
	bool found = false;
	pthread_mutex_lock(&cache_mutex);
	observation *entry = find_observation(device, name);
	if (entry && entry->property) {
		for (int i = 0; i < entry->property->count; i++) {
			if (!strcmp(entry->property->items[i].name, item_name)) {
				*result = entry->property->items[i];
				found = true;
				break;
			}
		}
	}
	pthread_mutex_unlock(&cache_mutex);
	return found;
}

static double number(const char *device, const char *name, const char *item_name) {
	indigo_item item;
	return item_lookup(device, name, item_name, &item) ? item.number.value : NAN;
}

static bool has_item(const char *device, const char *name, const char *item_name) {
	indigo_item item;
	return item_lookup(device, name, item_name, &item);
}

static bool switch_value(const char *device, const char *name, const char *item_name) {
	indigo_item item;
	return item_lookup(device, name, item_name, &item) && item.sw.value;
}

static bool change_switch(const char *device, const char *name, const char *item_name, bool value) {
	REQUIRE(WAIT_UNTIL(has_item(device, name, item_name), 10));
	REQUIRE(indigo_change_switch_property_1(&observer, device, name, item_name, value) == INDIGO_OK);
	return true;
}

static bool change_switch_and_wait(const char *device, const char *name, const char *item_name, bool value, indigo_property_state expected, double timeout) {
	unsigned before = revision(device, name);
	REQUIRE(change_switch(device, name, item_name, value));
	if (!WAIT_UNTIL(revision(device, name) > before && state(device, name) == (int)expected && switch_value(device, name, item_name) == value, timeout)) {
		fprintf(stderr, "%s.%s.%s = %d did not settle in state %d (state %d)\n", device, name, item_name, value, expected, state(device, name));
		dump_messages();
		return false;
	}
	return true;
}

static bool change_numbers(const char *device, const char *name, int count, const char **items, const double *values) {
	REQUIRE(WAIT_UNTIL(has_item(device, name, items[0]), 10));
	unsigned before = revision(device, name);
	REQUIRE(indigo_change_number_property(&observer, device, name, count, items, values) == INDIGO_OK);
	if (!WAIT_UNTIL(revision(device, name) > before && state(device, name) != INDIGO_BUSY_STATE, 10)) {
		fprintf(stderr, "%s.%s did not settle after a number change (state %d)\n", device, name, state(device, name));
		return false;
	}
	return true;
}

static bool change_number(const char *device, const char *name, const char *item_name, double value) {
	return change_numbers(device, name, 1, (const char *[]){ item_name }, (double []){ value });
}

static bool setting(const char *item_name, double value) {
	return change_number(AO, AGENT_AO_SETTINGS_PROPERTY_NAME, item_name, value);
}

static double setting_value(const char *item_name) {
	return number(AO, AGENT_AO_SETTINGS_PROPERTY_NAME, item_name);
}

static double stat(const char *item_name) {
	return number(AO, AGENT_AO_STATS_PROPERTY_NAME, item_name);
}

static bool select_device(const char *list, const char *device) {
	REQUIRE(WAIT_UNTIL(has_item(AO, list, device), 10));
	return change_switch_and_wait(AO, list, device, true, INDIGO_OK_STATE, 20);
}

// The process is done when the start property is no longer busy and the phase says how it ended
static bool run_process(const char *item_name, double timeout, int expected_phase) {
	unsigned before = revision(AO, AGENT_START_PROCESS_PROPERTY_NAME);
	REQUIRE(change_switch(AO, AGENT_START_PROCESS_PROPERTY_NAME, item_name, true));
	if (!WAIT_UNTIL(revision(AO, AGENT_START_PROCESS_PROPERTY_NAME) > before && state(AO, AGENT_START_PROCESS_PROPERTY_NAME) != INDIGO_BUSY_STATE && (int)stat("PHASE") == expected_phase, timeout)) {
		fprintf(stderr, "process %s ended in state %d, phase %g\n", item_name, state(AO, AGENT_START_PROCESS_PROPERTY_NAME), stat("PHASE"));
		return false;
	}
	return true;
}

static bool setup(bool select_ao) {
	indigo_set_log_level(getenv("INDIGO_TEST_DEBUG") ? INDIGO_LOG_DEBUG : INDIGO_LOG_ERROR);
	REQUIRE(indigo_start() == INDIGO_OK);
	bus_started = true;
	INDIGO_COPY_NAME(observer.name, "AO Agent observer");
	observer.version = INDIGO_VERSION_CURRENT;
	observer.define_property = defined;
	observer.update_property = updated;
	observer.delete_property = deleted;
	observer.send_message = message_received;
	REQUIRE(indigo_attach_client(&observer) == INDIGO_OK);
	observer_attached = true;
	REQUIRE(indigo_ccd_simulator(INDIGO_DRIVER_INIT, NULL) == INDIGO_OK);
	ccd_started = true;
	REQUIRE(indigo_agent_ao(INDIGO_DRIVER_INIT, NULL) == INDIGO_OK);
	ao_started = true;
	// the setup of the simulated sky is a property of the guidescope camera
	REQUIRE(change_switch_and_wait(SETUP_CAMERA, CONNECTION_PROPERTY_NAME, CONNECTION_CONNECTED_ITEM_NAME, true, INDIGO_OK_STATE, 20));
	REQUIRE(change_numbers(SETUP_CAMERA, "SIMULATION_SETUP", 8, (const char *[]){ "PER_ERR_CYCLE", "AO_ANGLE", "AO_SCALE", "AO_LIMIT", "AO_TARGET", "AO_MOVE_TIME", "TT_RMS", "SEED" }, (double []){ 0, 30, 0.5, 100, 0, 0, 0, 4711 }));
	REQUIRE(select_device(FILTER_CCD_LIST_PROPERTY_NAME, OAG_CAMERA));
	if (select_ao) {
		REQUIRE(select_device(FILTER_AO_LIST_PROPERTY_NAME, AO_DEVICE));
	}
	return true;
}

static void cleanup(void) {
	if (ao_started) {
		indigo_agent_ao(INDIGO_DRIVER_SHUTDOWN, NULL);
	}
	if (ccd_started) {
		indigo_ccd_simulator(INDIGO_DRIVER_SHUTDOWN, NULL);
	}
	if (observer_attached) {
		indigo_detach_client(&observer);
	}
	if (bus_started) {
		indigo_stop();
	}
}

// The axes of the simulated AO: one step along the RA axis moves the star by AO_SCALE pixels along the AO angle,
// the Dec axis is perpendicular to it. The calibration of the AO measures them through the camera.
static bool set_ideal_calibration(void) {
	double scale = 0.5, angle = 30 * M_PI / 180;
	REQUIRE(change_numbers(AO, AGENT_AO_SETTINGS_PROPERTY_NAME, 4, (const char *[]){ "RA_X", "RA_Y", "DEC_X", "DEC_Y" }, (double []){ scale * cos(angle), scale * sin(angle), -scale * sin(angle), scale * cos(angle) }));
	return true;
}

// The exercise moves the AO to four corners and the centre of its range and centers it again
static void exercise_visits_the_corners_and_centers(void) {
	CHECK(setting("EXERCISE_CYCLES", 1));
	CHECK(setting("EXERCISE_INTERVAL", 0.2));
	unsigned ra_moves = state_count(AO, AO_GUIDE_RA_PROPERTY_NAME, INDIGO_BUSY_STATE) + state_count(AO, AO_GUIDE_RA_PROPERTY_NAME, INDIGO_OK_STATE);
	CHECK(run_process(AGENT_AO_EXERCISE_ITEM_NAME, 60, AGENT_AO_PHASE_DONE));
	CHECK(state_count(AO, AO_GUIDE_RA_PROPERTY_NAME, INDIGO_OK_STATE) + state_count(AO, AO_GUIDE_RA_PROPERTY_NAME, INDIGO_BUSY_STATE) >= ra_moves + 4);
	CHECK(state_count(AO, AO_GUIDE_DEC_PROPERTY_NAME, INDIGO_OK_STATE) + state_count(AO, AO_GUIDE_DEC_PROPERTY_NAME, INDIGO_BUSY_STATE) >= 4);
	CHECK(fabs(stat("POSITION_RA")) < 1e-6 && fabs(stat("POSITION_DEC")) < 1e-6);
	CHECK(state(AO, AGENT_START_PROCESS_PROPERTY_NAME) == INDIGO_OK_STATE);
}

// The AO is moved along both axes while the OAG measures the star, the result is the matrix of the simulated AO
static void calibration_recovers_the_simulated_axes(void) {
	CHECK(setting("CALIBRATION_STEP", 10));
	CHECK(setting("CALIBRATION_FRAMES", 3));
	CHECK(run_process(AGENT_AO_CALIBRATION_ITEM_NAME, 120, AGENT_AO_PHASE_DONE));
	double angle = 30 * M_PI / 180, scale = 0.5;
	CHECK(fabs(setting_value("RA_X") - scale * cos(angle)) < 0.1);
	CHECK(fabs(setting_value("RA_Y") - scale * sin(angle)) < 0.1);
	CHECK(fabs(setting_value("DEC_X") + scale * sin(angle)) < 0.1);
	CHECK(fabs(setting_value("DEC_Y") - scale * cos(angle)) < 0.1);
	CHECK(fabs(setting_value("SCALE_RA") - scale) < 0.1);
	CHECK(fabs(setting_value("SCALE_DEC") - scale) < 0.1);
	CHECK(fabs(setting_value("ANGLE") + 30) < 8);
	CHECK(fabs(stat("POSITION_RA")) < 1e-6 && fabs(stat("POSITION_DEC")) < 1e-6);
}

// A slow tip-tilt of 2 px RMS moves the star of the OAG, the closed loop leaves a fraction of it
static void loop_cancels_slow_tip_tilt(void) {
	CHECK(set_ideal_calibration());
	CHECK(change_numbers(SETUP_CAMERA, "SIMULATION_SETUP", 3, (const char *[]){ "TT_RMS", "TT_FREQ", "SEED" }, (double []){ 2.0, 0.3, 4711 }));
	CHECK(setting("EXPOSURE", 0.05));
	unsigned before = revision(AO, AGENT_START_PROCESS_PROPERTY_NAME);
	CHECK(change_switch(AO, AGENT_START_PROCESS_PROPERTY_NAME, AGENT_AO_TRACKING_ITEM_NAME, true));
	CHECK(WAIT_UNTIL(revision(AO, AGENT_START_PROCESS_PROPERTY_NAME) > before && (int)stat("PHASE") == AGENT_AO_PHASE_TRACKING, 20));
	CHECK(WAIT_UNTIL(stat("FRAME_RATE") > 3, 20));
	indigo_usleep(15000000);
	double rmse_x = stat("RMSE_X"), rmse_y = stat("RMSE_Y");
	printf("    loop on: residual RMSE %.3f, %.3f px at %.1f Hz, AO position %.1f, %.1f\n", rmse_x, rmse_y, stat("FRAME_RATE"), stat("POSITION_RA"), stat("POSITION_DEC"));
	CHECK(rmse_x < 1.2 && rmse_y < 1.2);
	CHECK(change_switch(AO, AGENT_ABORT_PROCESS_PROPERTY_NAME, AGENT_ABORT_PROCESS_ITEM_NAME, true));
	CHECK(WAIT_UNTIL(state(AO, AGENT_START_PROCESS_PROPERTY_NAME) != INDIGO_BUSY_STATE, 20));
	CHECK(WAIT_UNTIL(fabs(stat("POSITION_RA")) < 1e-6 && fabs(stat("POSITION_DEC")) < 1e-6, 10));
}

// An AO that is not calibrated is not driven
static void loop_refuses_an_uncalibrated_ao(void) {
	unsigned before = revision(AO, AGENT_START_PROCESS_PROPERTY_NAME);
	CHECK(change_switch(AO, AGENT_START_PROCESS_PROPERTY_NAME, AGENT_AO_TRACKING_ITEM_NAME, true));
	CHECK(WAIT_UNTIL(revision(AO, AGENT_START_PROCESS_PROPERTY_NAME) > before && state(AO, AGENT_START_PROCESS_PROPERTY_NAME) == INDIGO_ALERT_STATE, 20));
	CHECK(has_message("not calibrated"));
}

// Without an AO the processes fail instead of waiting
static void processes_refuse_without_an_ao(void) {
	unsigned before = revision(AO, AGENT_START_PROCESS_PROPERTY_NAME);
	CHECK(change_switch(AO, AGENT_START_PROCESS_PROPERTY_NAME, AGENT_AO_CENTER_ITEM_NAME, true));
	CHECK(WAIT_UNTIL(revision(AO, AGENT_START_PROCESS_PROPERTY_NAME) > before && state(AO, AGENT_START_PROCESS_PROPERTY_NAME) == INDIGO_ALERT_STATE, 20));
	CHECK(has_message("No active optics selected"));
}

static const indigo_test_case tests[] = {
	{ "exercise visits the corners and centers", exercise_visits_the_corners_and_centers },
	{ "calibration recovers the simulated axes", calibration_recovers_the_simulated_axes },
	{ "loop cancels slow tip-tilt", loop_cancels_slow_tip_tilt },
	{ "loop refuses an uncalibrated AO", loop_refuses_an_uncalibrated_ao },
	{ "processes refuse without an AO", processes_refuse_without_an_ao }
};

int main(int argc, char **argv) {
	setvbuf(stdout, NULL, _IONBF, 0);
	int result = 0, executed = 0, passed = 0;
	for (int i = 0; i < (int)ARRAY_SIZE(tests); i++) {
		if (argc > 1 && !strstr(tests[i].name, argv[1])) {
			continue;
		}
		executed++;
		pid_t child = fork();
		if (child == 0) {
			alarm(420);
			if (!indigo_test_use_private_home()) {
				exit(1);
			}
			bool ready = setup(tests[i].function != processes_refuse_without_an_ao);
			int status = ready ? indigo_run_tests("AO Agent with the CCD simulator", tests + i, 1) : 1;
			cleanup();
			indigo_test_remove_private_home();
			exit(status || indigo_test_failures ? 1 : 0);
		}
		int status = -1;
		if (child < 0 || waitpid(child, &status, 0) != child || !WIFEXITED(status) || WEXITSTATUS(status)) {
			fprintf(stderr, "Failed case: %s (status %d)\n", tests[i].name, status);
			result = 1;
		} else {
			passed++;
		}
	}
	printf("AO Agent with the CCD simulator: %d/%d cases passed (including cleanup)\n", passed, executed);
	return executed ? result : 2;
}
