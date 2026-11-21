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

#include <string.h>
#include <assert.h>

#include <indigo/indigo_bus.h>
#include <indigo/indigo_names.h>
#include <indigo/indigo_ao_driver.h>
#include <indigo/indigo_filter.h>

#include "../test_runner.h"

#define TEST_DRIVER_NAME "filter-ao-list-test"
#define TEST_AO_NAME "Filter AO Test AO"
#define TEST_AGENT_NAME "Filter AO Test Agent"
#define WAIT_STEPS 500

static volatile bool ao_list_hidden_by_default = false;
static volatile bool ao_list_defined = false;
static volatile bool ao_list_has_device = false;
static volatile bool ao_proxy_defined = false;
static volatile int ao_north_requests = 0;
static volatile double ao_north_value = 0;
static char ao_proxy_group[INDIGO_NAME_SIZE] = { 0 };

static indigo_result test_ao_attach(indigo_device *device) {
	if (indigo_ao_attach(device, TEST_DRIVER_NAME, 0x0001) == INDIGO_OK) {
		AO_GUIDE_DEC_PROPERTY->hidden = false;
		AO_GUIDE_RA_PROPERTY->hidden = false;
		AO_RESET_PROPERTY->hidden = false;
		return indigo_ao_enumerate_properties(device, NULL, NULL);
	}
	return INDIGO_FAILED;
}

static indigo_result test_ao_change_property(indigo_device *device, indigo_client *client, indigo_property *property) {
	if (indigo_property_match_changeable(CONNECTION_PROPERTY, property)) {
		indigo_property_copy_values(CONNECTION_PROPERTY, property, false);
		CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
		return indigo_ao_change_property(device, NULL, CONNECTION_PROPERTY);
	} else if (indigo_property_match_changeable(AO_GUIDE_DEC_PROPERTY, property)) {
		indigo_property_copy_values(AO_GUIDE_DEC_PROPERTY, property, false);
		ao_north_value = AO_GUIDE_NORTH_ITEM->number.value;
		ao_north_requests++;
		AO_GUIDE_NORTH_ITEM->number.value = AO_GUIDE_SOUTH_ITEM->number.value = 0;
		AO_GUIDE_DEC_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, AO_GUIDE_DEC_PROPERTY, NULL);
		return INDIGO_OK;
	}
	return indigo_ao_change_property(device, client, property);
}

static indigo_result test_ao_detach(indigo_device *device) {
	return indigo_ao_detach(device);
}

static indigo_result test_agent_attach(indigo_device *device) {
	if (indigo_filter_device_attach(device, TEST_DRIVER_NAME, 0x0001, INDIGO_INTERFACE_AUX) == INDIGO_OK) {
		ao_list_hidden_by_default = FILTER_AO_LIST_PROPERTY->hidden && FILTER_RELATED_AO_LIST_PROPERTY->hidden;
		FILTER_AO_LIST_PROPERTY->hidden = false;
		return indigo_filter_enumerate_properties(device, NULL, NULL);
	}
	return INDIGO_FAILED;
}

static indigo_result test_agent_detach(indigo_device *device) {
	indigo_cancel_pending_handlers(device);
	indigo_cancel_all_timers(device);
	return indigo_filter_device_detach(device);
}

static indigo_result observe(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	if (!strcmp(property->device, TEST_AGENT_NAME)) {
		if (!strcmp(property->name, FILTER_AO_LIST_PROPERTY_NAME)) {
			ao_list_defined = true;
			for (int i = 0; i < property->count; i++) {
				if (!strcmp(property->items[i].name, TEST_AO_NAME)) {
					ao_list_has_device = true;
				}
			}
		} else if (!strcmp(property->name, AO_GUIDE_DEC_PROPERTY_NAME)) {
			INDIGO_COPY_NAME(ao_proxy_group, property->group);
			ao_proxy_defined = true;
		}
	}
	return INDIGO_OK;
}

static bool wait_for(volatile bool *flag) {
	for (int i = 0; i < WAIT_STEPS && !*flag; i++) {
		indigo_usleep(10000);
	}
	return *flag;
}

static void ao_device_is_listed_selected_and_forwarded(void) {
	static indigo_device ao_device = INDIGO_DEVICE_INITIALIZER(TEST_AO_NAME, test_ao_attach, indigo_ao_enumerate_properties, test_ao_change_property, NULL, test_ao_detach);
	static indigo_device agent_device = INDIGO_DEVICE_INITIALIZER(TEST_AGENT_NAME, test_agent_attach, indigo_filter_enumerate_properties, indigo_filter_change_property, NULL, test_agent_detach);
	static indigo_client agent_client = { TEST_AGENT_NAME, false, NULL, INDIGO_OK, INDIGO_VERSION_CURRENT, NULL, indigo_filter_client_attach, indigo_filter_define_property, indigo_filter_update_property, indigo_filter_delete_property, NULL, indigo_filter_client_detach };
	static indigo_client observer = { "Filter AO observer", false, NULL, INDIGO_OK, INDIGO_VERSION_CURRENT, NULL, NULL, observe, observe, NULL, NULL, NULL };
	ASSERT_EQ_INT(14, INDIGO_FILTER_LIST_COUNT);
	ASSERT_EQ_INT(13, INDIGO_FILTER_AO_INDEX);
	ASSERT_EQ_INT(INDIGO_OK, indigo_start());
	ASSERT_EQ_INT(INDIGO_OK, indigo_attach_client(&observer));
	ASSERT_EQ_INT(INDIGO_OK, indigo_attach_device(&ao_device));
	ASSERT_EQ_INT(INDIGO_OK, indigo_attach_device(&agent_device));
	agent_client.client_context = agent_device.device_context;
	ASSERT_EQ_INT(INDIGO_OK, indigo_attach_client(&agent_client));
	// the AO list and its related list are hidden until an agent unhides them
	ASSERT_TRUE(ao_list_hidden_by_default);
	ASSERT_TRUE(wait_for(&ao_list_defined));
	// an AO-only device is offered by the AO list
	ASSERT_TRUE(wait_for(&ao_list_has_device));
	// selecting it connects the device and proxies its properties with their own AO_ names
	ASSERT_EQ_INT(INDIGO_OK, indigo_change_switch_property_1(&observer, TEST_AGENT_NAME, FILTER_AO_LIST_PROPERTY_NAME, TEST_AO_NAME, true));
	ASSERT_TRUE(wait_for(&ao_proxy_defined));
	ASSERT_STREQ("AO Guider", ao_proxy_group);
	// a change request sent to the agent reaches the selected AO device
	ASSERT_EQ_INT(INDIGO_OK, indigo_change_number_property_1(&observer, TEST_AGENT_NAME, AO_GUIDE_DEC_PROPERTY_NAME, AO_GUIDE_NORTH_ITEM_NAME, 7));
	for (int i = 0; i < WAIT_STEPS && ao_north_requests == 0; i++) {
		indigo_usleep(10000);
	}
	ASSERT_EQ_INT(1, ao_north_requests);
	ASSERT_NEAR(7, ao_north_value, 1e-9);
	ASSERT_EQ_INT(INDIGO_OK, indigo_detach_client(&agent_client));
	ASSERT_EQ_INT(INDIGO_OK, indigo_detach_device(&agent_device));
	ASSERT_EQ_INT(INDIGO_OK, indigo_detach_device(&ao_device));
	ASSERT_EQ_INT(INDIGO_OK, indigo_detach_client(&observer));
	ASSERT_EQ_INT(INDIGO_OK, indigo_stop());
}

int main(void) {
	const indigo_test_case tests[] = {
		{ "ao_device_is_listed_selected_and_forwarded", ao_device_is_listed_selected_and_forwarded }
	};
	return indigo_run_tests("filter AO list unit tests", tests, (int)(sizeof(tests) / sizeof(tests[0])));
}
