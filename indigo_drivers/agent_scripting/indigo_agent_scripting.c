// Copyright (c) 2020 CloudMakers, s. r. o.
// All rights reserved.
//
// You can use this software under the terms of 'INDIGO Astronomy
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

// version history
// 2.0 by Peter Polakovic <peter.polakovic@cloudmakers.eu>

/** INDIGO Scripting agent
 \file indigo_agent_scripting.c
 */

#define DRIVER_VERSION 0x0001
#define DRIVER_NAME	"indigo_agent_scripting"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>

#include <indigo/indigo_bus.h>
#include <indigo/indigo_driver.h>

#include "duktape.h"
#include "indigo_agent_scripting.h"

#define SCRIPT_GROUP															"Scripts"
#define PRIVATE_DATA															private_data

#define MAX_USER_SCRIPT_COUNT											128

#define AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY		(PRIVATE_DATA->agent_on_load_script_property)
#define AGENT_SCRIPTING_ON_LOAD_SCRIPT_ITEM				(AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY->items+0)

#define AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY	(PRIVATE_DATA->agent_on_unload_script_property)
#define AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_ITEM			(AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY->items+0)

#define AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY				(PRIVATE_DATA->agent_add_script_property)
#define AGENT_SCRIPTING_ADD_SCRIPT_NAME_ITEM			(AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY->items+0)
#define AGENT_SCRIPTING_ADD_SCRIPT_ITEM						(AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY->items+1)

#define AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY		(PRIVATE_DATA->agent_delete_script_property)
#define AGENT_SCRIPTING_DELETE_SCRIPT_NAME_ITEM		(AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY->items+0)

#define AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY		(PRIVATE_DATA->agent_execute_script_property)
#define AGENT_SCRIPTING_EXECUTE_SCRIPT_NAME_ITEM	(AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY->items+0)

#define AGENT_SCRIPTING_SCRIPT_PROPERTY(i)				(PRIVATE_DATA->agent_scripts_property[i])
#define AGENT_SCRIPTING_SCRIPT_NAME_ITEM(i)				(AGENT_SCRIPTING_SCRIPT_PROPERTY(i)->items+0)
#define AGENT_SCRIPTING_SCRIPT_ITEM(i)						(AGENT_SCRIPTING_SCRIPT_PROPERTY(i)->items+1)

static char boot_js[] = {
#include "boot.js.dat"
	0
};

typedef struct {
	indigo_property *agent_on_load_script_property;
	indigo_property *agent_on_unload_script_property;
	indigo_property *agent_add_script_property;
	indigo_property *agent_delete_script_property;
	indigo_property *agent_execute_script_property;
	indigo_property *agent_scripts_property[MAX_USER_SCRIPT_COUNT];
	duk_context *ctx;
	pthread_mutex_t mutex;
} agent_private_data;

static agent_private_data *private_data = NULL;
static indigo_device *agent_device = NULL;
static indigo_client *agent_client = NULL;

static void save_config(indigo_device *device) {
	if (pthread_mutex_trylock(&DEVICE_CONTEXT->config_mutex) == 0) {
		pthread_mutex_unlock(&DEVICE_CONTEXT->config_mutex);
		indigo_save_property(device, NULL, AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY);
		indigo_save_property(device, NULL, AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY);
		for (int i = 0; i < MAX_USER_SCRIPT_COUNT; i++) {
			indigo_property *script_property = AGENT_SCRIPTING_SCRIPT_PROPERTY(i);
			if (script_property) {
				char name[INDIGO_NAME_SIZE];
				indigo_copy_name(name, script_property->name);
				indigo_copy_name(script_property->name, AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY_NAME);
				indigo_save_property(device, NULL, script_property);
				indigo_copy_name(script_property->name, name);
			}
		}
		if (DEVICE_CONTEXT->property_save_file_handle) {
			CONFIG_PROPERTY->state = INDIGO_OK_STATE;
			close(DEVICE_CONTEXT->property_save_file_handle);
			DEVICE_CONTEXT->property_save_file_handle = 0;
		} else {
			CONFIG_PROPERTY->state = INDIGO_ALERT_STATE;
		}
		CONFIG_SAVE_ITEM->sw.value = false;
		indigo_update_property(device, CONFIG_PROPERTY, NULL);
	}
}

// -------------------------------------------------------------------------------- Duktape bindings

static duk_ret_t error_message(duk_context *ctx) {
	const char *message = duk_to_string(ctx, 0);
	indigo_error(message);
	return 0;
}

static duk_ret_t log_message(duk_context *ctx) {
	const char *message = duk_to_string(ctx, 0);
	indigo_log(message);
	return 0;
}

static duk_ret_t debug_message(duk_context *ctx) {
	const char *message = duk_to_string(ctx, 0);
	indigo_debug(message);
	return 0;
}

static duk_ret_t trace_message(duk_context *ctx) {
	const char *message = duk_to_string(ctx, 0);
	indigo_trace(message);
	return 0;
}



static duk_ret_t send_message(duk_context *ctx) {
	const char *message = duk_to_string(ctx, 0);
	indigo_send_message(agent_device, message);
	return 0;
}

static duk_ret_t emumerate_properties(duk_context *ctx) {
	const char *device = duk_to_string(ctx, 0);
	const char *property = duk_to_string(ctx, 1);
	indigo_property property_template = { 0 };
	indigo_copy_name(property_template.device, device);
	indigo_copy_name(property_template.name, property);
	indigo_enumerate_properties(agent_client, &property_template);
	return 0;
}

static duk_ret_t enable_blob(duk_context *ctx) {
	const char *device = duk_to_string(ctx, 0);
	const char *property = duk_to_string(ctx, 1);
	const bool state = duk_to_boolean(ctx, 2);
	indigo_property property_template = { 0 };
	indigo_copy_name(property_template.device, device);
	indigo_copy_name(property_template.name, property);
	indigo_enable_blob(agent_client, &property_template, state);
	return 0;
}

static duk_ret_t change_text_property(duk_context *ctx) {
	const char *device = duk_to_string(ctx, 0);
	const char *property = duk_to_string(ctx, 1);
	char *names[INDIGO_MAX_ITEMS];
	char *values[INDIGO_MAX_ITEMS];
	duk_enum(ctx, 2, DUK_ENUM_OWN_PROPERTIES_ONLY );
	int i = 0;
	while (duk_next(ctx, -1, true)) {
		const char *key = duk_to_string(ctx, -2);
		const char *value = duk_to_string(ctx, -1);
		names[i] = strdup(key);
		values[i] = strdup(value);
		duk_pop_2(ctx);
		i++;
	}
	indigo_change_text_property(agent_client, device, property, i, (const char **)names, (const char **)values);
	for (int j = 0; j < i; j++) {
		if (names[j])
			free(names[j]);
		if (values[j])
			free(values[j]);
	}
	return 0;
}

static duk_ret_t change_number_property(duk_context *ctx) {
	const char *device = duk_to_string(ctx, 0);
	const char *property = duk_to_string(ctx, 1);
	char *names[INDIGO_MAX_ITEMS];
	double values[INDIGO_MAX_ITEMS];
	duk_enum(ctx, 2, DUK_ENUM_OWN_PROPERTIES_ONLY );
	int i = 0;
	while (duk_next(ctx, -1, true)) {
		const char *key = duk_to_string(ctx, -2);
		double value = duk_to_number(ctx, -1);
		names[i] = strdup(key);
		values[i] = value;
		duk_pop_2(ctx);
		i++;
	}
	indigo_change_number_property(agent_client, device, property, i, (const char **)names, (const double *)values);
	for (int j = 0; j < i; j++) {
		if (names[j])
			free(names[j]);
	}
	return 0;
}

static duk_ret_t change_switch_property(duk_context *ctx) {
	const char *device = duk_to_string(ctx, 0);
	const char *property = duk_to_string(ctx, 1);
	char *names[INDIGO_MAX_ITEMS];
	bool values[INDIGO_MAX_ITEMS];
	duk_enum(ctx, 2, DUK_ENUM_OWN_PROPERTIES_ONLY );
	int i = 0;
	while (duk_next(ctx, -1, true)) {
		const char *key = duk_to_string(ctx, -2);
		double value = duk_to_boolean(ctx, -1);
		names[i] = strdup(key);
		values[i] = value;
		duk_pop_2(ctx);
		i++;
	}
	indigo_change_switch_property(agent_client, device, property, i, (const char **)names, (const bool *)values);
	for (int j = 0; j < i; j++) {
		if (names[j])
			free(names[j]);
	}
	return 0;
}

static void execute_script(indigo_property *property) {
	property->state = INDIGO_BUSY_STATE;
	indigo_update_property(agent_device, property, NULL);
	char *script = indigo_get_text_item_value(property->count == 1 ? property->items : property->items + 1);
	if (script && *script) {
		pthread_mutex_lock(&PRIVATE_DATA->mutex);
		if (duk_peval_string(PRIVATE_DATA->ctx, script)) {
			indigo_send_message(agent_device, "Failed to execute script '%s' (%s)", property->label, duk_safe_to_string(PRIVATE_DATA->ctx, -1));
			property->state = INDIGO_ALERT_STATE;
		} else {
			property->state = INDIGO_OK_STATE;
		}
		pthread_mutex_unlock(&PRIVATE_DATA->mutex);
	} else {
		property->state = INDIGO_OK_STATE;
	}
	indigo_update_property(agent_device, property, NULL);
}

// -------------------------------------------------------------------------------- INDIGO agent device implementation

static indigo_result agent_enumerate_properties(indigo_device *device, indigo_client *client, indigo_property *property);

static indigo_result agent_device_attach(indigo_device *device) {
	assert(device != NULL);
	assert(PRIVATE_DATA != NULL);
	if (indigo_device_attach(device, DRIVER_NAME, DRIVER_VERSION, INDIGO_INTERFACE_AGENT) == INDIGO_OK) {
		// -------------------------------------------------------------------------------- Script properties
		AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY = indigo_init_text_property(NULL, device->name, AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY_NAME, SCRIPT_GROUP, "On load", INDIGO_OK_STATE, INDIGO_RW_PERM, 1);
		if (AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY == NULL)
			return INDIGO_FAILED;
		indigo_init_text_item(AGENT_SCRIPTING_ON_LOAD_SCRIPT_ITEM, AGENT_SCRIPTING_ON_LOAD_SCRIPT_ITEM_NAME, "Script", "");
		AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY = indigo_init_text_property(NULL, device->name, AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY_NAME, SCRIPT_GROUP, "On unload", INDIGO_OK_STATE, INDIGO_RW_PERM, 1);
		if (AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY == NULL)
			return INDIGO_FAILED;
		indigo_init_text_item(AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_ITEM, AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_ITEM_NAME, "Script", "");
		AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY = indigo_init_text_property(NULL, device->name, AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY_NAME, MAIN_GROUP, "Add script", INDIGO_OK_STATE, INDIGO_RW_PERM, 2);
		if (AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY == NULL)
			return INDIGO_FAILED;
		indigo_init_text_item(AGENT_SCRIPTING_ADD_SCRIPT_NAME_ITEM, AGENT_SCRIPTING_ADD_SCRIPT_NAME_ITEM_NAME, "Name", "");
		indigo_init_text_item(AGENT_SCRIPTING_ADD_SCRIPT_ITEM, AGENT_SCRIPTING_ADD_SCRIPT_ITEM_NAME, "Script", "");
		AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY = indigo_init_text_property(NULL, device->name, AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY_NAME, MAIN_GROUP, "Delete script", INDIGO_OK_STATE, INDIGO_RW_PERM, 1);
		if (AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY == NULL)
			return INDIGO_FAILED;
		indigo_init_text_item(AGENT_SCRIPTING_DELETE_SCRIPT_NAME_ITEM, AGENT_SCRIPTING_DELETE_SCRIPT_NAME_ITEM_NAME, "Name", "");
		AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY = indigo_init_text_property(NULL, device->name, AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY_NAME, MAIN_GROUP, "Execute script", INDIGO_OK_STATE, INDIGO_RW_PERM, 1);
		if (AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY == NULL)
			return INDIGO_FAILED;
		indigo_init_text_item(AGENT_SCRIPTING_EXECUTE_SCRIPT_NAME_ITEM, AGENT_SCRIPTING_EXECUTE_SCRIPT_NAME_ITEM_NAME, "Name", "");
		// --------------------------------------------------------------------------------
		CONNECTION_PROPERTY->hidden = true;
		CONFIG_PROPERTY->hidden = true;
		PROFILE_PROPERTY->hidden = true;
		PRIVATE_DATA->ctx = duk_create_heap_default();
		if (PRIVATE_DATA->ctx) {
			duk_push_c_function(PRIVATE_DATA->ctx, error_message, 1);
			duk_put_global_string(PRIVATE_DATA->ctx, "indigo_error");
			duk_push_c_function(PRIVATE_DATA->ctx, log_message, 1);
			duk_put_global_string(PRIVATE_DATA->ctx, "indigo_log");
			duk_push_c_function(PRIVATE_DATA->ctx, debug_message, 1);
			duk_put_global_string(PRIVATE_DATA->ctx, "indigo_debug");
			duk_push_c_function(PRIVATE_DATA->ctx, trace_message, 1);
			duk_put_global_string(PRIVATE_DATA->ctx, "indigo_trace");
			duk_push_c_function(PRIVATE_DATA->ctx, send_message, 1);
			duk_put_global_string(PRIVATE_DATA->ctx, "indigo_send_message");
			duk_push_c_function(PRIVATE_DATA->ctx, emumerate_properties, 2);
			duk_put_global_string(PRIVATE_DATA->ctx, "indigo_enumerate_properties");
			duk_push_c_function(PRIVATE_DATA->ctx, enable_blob, 3);
			duk_put_global_string(PRIVATE_DATA->ctx, "indigo_enable_blob");
			duk_push_c_function(PRIVATE_DATA->ctx, change_text_property, 3);
			duk_put_global_string(PRIVATE_DATA->ctx, "indigo_change_text_property");
			duk_push_c_function(PRIVATE_DATA->ctx, change_number_property, 3);
			duk_put_global_string(PRIVATE_DATA->ctx, "indigo_change_number_property");
			duk_push_c_function(PRIVATE_DATA->ctx, change_switch_property, 3);
			duk_put_global_string(PRIVATE_DATA->ctx, "indigo_change_switch_property");
			if (duk_peval_string(PRIVATE_DATA->ctx, boot_js)) {
				INDIGO_DRIVER_ERROR(DRIVER_NAME, "%s", duk_safe_to_string(PRIVATE_DATA->ctx, -1));
			}
		}
		pthread_mutex_init(&PRIVATE_DATA->mutex, NULL);
		INDIGO_DEVICE_ATTACH_LOG(DRIVER_NAME, device->name);
		return agent_enumerate_properties(device, NULL, NULL);
	}
	return INDIGO_FAILED;
}

static indigo_result agent_enumerate_properties(indigo_device *device, indigo_client *client, indigo_property *property) {
	if (indigo_property_match(AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY, property))
		indigo_define_property(device, AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY, NULL);
	if (indigo_property_match(AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY, property))
		indigo_define_property(device, AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY, NULL);
	if (indigo_property_match(AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY, property))
		indigo_define_property(device, AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY, NULL);
	if (indigo_property_match(AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY, property))
		indigo_define_property(device, AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY, NULL);
	if (indigo_property_match(AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY, property))
		indigo_define_property(device, AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY, NULL);
	for (int i = 0; i < MAX_USER_SCRIPT_COUNT; i++) {
		indigo_property *script_property = AGENT_SCRIPTING_SCRIPT_PROPERTY(i);
		if (script_property)
			indigo_define_property(device, script_property, NULL);
	}
	return indigo_device_enumerate_properties(device, client, property);
}

static indigo_result agent_change_property(indigo_device *device, indigo_client *client, indigo_property *property) {
	assert(device != NULL);
	assert(DEVICE_CONTEXT != NULL);
	assert(property != NULL);
	if (indigo_property_match(CONFIG_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CONFIG
		if (indigo_switch_match(CONFIG_LOAD_ITEM, property)) {
			indigo_device_change_property(device, client, property);
			execute_script(AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY);
			return INDIGO_OK;
		}
	} else if (indigo_property_match(AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- AGENT_SCRIPTING_ON_LOAD_SCRIPT
		indigo_property_copy_values(AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY, property, false);
		AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY, NULL);
		save_config(device);
		return INDIGO_OK;
	} else if (indigo_property_match(AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- AGENT_SCRIPTING_ON_UNLOAD_SCRIPT
		indigo_property_copy_values(AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY, property, false);
		AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY, NULL);
		save_config(device);
		return INDIGO_OK;
	} else if (indigo_property_match(AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- AGENT_SCRIPTING_ADD_SCRIPT
		indigo_property_copy_values(AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY, property, false);
		int empty_slot = -1;
		for (int i = 0; i < MAX_USER_SCRIPT_COUNT; i++) {
			indigo_property *script_property = AGENT_SCRIPTING_SCRIPT_PROPERTY(i);
			if (script_property) {
				if (!strcmp(script_property->items[0].text.value, AGENT_SCRIPTING_ADD_SCRIPT_NAME_ITEM->text.value)) {
					AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY->state = INDIGO_ALERT_STATE;
					indigo_update_property(device, AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY, "Script %s already exists", AGENT_SCRIPTING_ADD_SCRIPT_NAME_ITEM->text.value);
					return INDIGO_OK;
				}
			} else if (empty_slot == -1) {
				empty_slot = i;
			}
		}
		if (empty_slot == -1) {
			AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY->state = INDIGO_ALERT_STATE;
			indigo_update_property(device, AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY, "Too many scripts defined");
			return INDIGO_OK;
		} else {
			char name[INDIGO_NAME_SIZE];
			sprintf(name, AGENT_SCRIPTING_SCRIPT_PROPERTY_NAME, empty_slot);
			indigo_property *script_property = AGENT_SCRIPTING_SCRIPT_PROPERTY(empty_slot) = indigo_init_text_property(NULL, device->name, name, SCRIPT_GROUP, AGENT_SCRIPTING_ADD_SCRIPT_NAME_ITEM->text.value, INDIGO_OK_STATE, INDIGO_RW_PERM, 2);
			indigo_init_text_item(script_property->items + 0, AGENT_SCRIPTING_SCRIPT_NAME_ITEM_NAME, "Name", AGENT_SCRIPTING_ADD_SCRIPT_NAME_ITEM->text.value);
			indigo_init_text_item(script_property->items + 1, AGENT_SCRIPTING_SCRIPT_ITEM_NAME, "Script", indigo_get_text_item_value(AGENT_SCRIPTING_ADD_SCRIPT_ITEM));
			indigo_define_property(device, script_property, NULL);
		}
		indigo_set_text_item_value(AGENT_SCRIPTING_ADD_SCRIPT_NAME_ITEM, "");
		indigo_set_text_item_value(AGENT_SCRIPTING_ADD_SCRIPT_ITEM, "");
		AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY->state = INDIGO_OK_STATE;
		indigo_update_property(device, AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY, NULL);
		save_config(device);
		return INDIGO_OK;
	} else if (indigo_property_match(AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- AGENT_SCRIPTING_DELETE_SCRIPT
		indigo_property_copy_values(AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY, property, false);
		for (int i = 0; i < MAX_USER_SCRIPT_COUNT; i++) {
			indigo_property *script_property = AGENT_SCRIPTING_SCRIPT_PROPERTY(i);
			if (script_property) {
				if (!strcmp(script_property->items[0].text.value, AGENT_SCRIPTING_DELETE_SCRIPT_NAME_ITEM->text.value)) {
					indigo_delete_property(device, script_property, NULL);
					indigo_release_property(script_property);
					AGENT_SCRIPTING_SCRIPT_PROPERTY(i) = NULL;
					indigo_set_text_item_value(AGENT_SCRIPTING_DELETE_SCRIPT_NAME_ITEM, "");
					AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY->state = INDIGO_OK_STATE;
					indigo_update_property(device, AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY, NULL);
					save_config(device);
					return INDIGO_OK;
				}
			}
		}
		AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY->state = INDIGO_ALERT_STATE;
		indigo_update_property(device, AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY, "Script %s doesn't exists", AGENT_SCRIPTING_DELETE_SCRIPT_NAME_ITEM->text.value);
		return INDIGO_OK;
	} else if (indigo_property_match(AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- AGENT_SCRIPTING_EXECUTE_SCRIPT
		indigo_property_copy_values(AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY, property, false);
		for (int i = 0; i < MAX_USER_SCRIPT_COUNT; i++) {
			indigo_property *script_property = AGENT_SCRIPTING_SCRIPT_PROPERTY(i);
			if (script_property) {
				if (!strcmp(script_property->items[0].text.value, AGENT_SCRIPTING_EXECUTE_SCRIPT_NAME_ITEM->text.value)) {
					execute_script(script_property);
					return INDIGO_OK;
				}
			}
		}
		AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY->state = INDIGO_ALERT_STATE;
		indigo_update_property(device, AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY, "Script %s doesn't exists", AGENT_SCRIPTING_EXECUTE_SCRIPT_NAME_ITEM->text.value);
		return INDIGO_OK;
	} else {
		for (int i = 0; i < MAX_USER_SCRIPT_COUNT; i++) {
			indigo_property *script_property = AGENT_SCRIPTING_SCRIPT_PROPERTY(i);
			if (script_property && indigo_property_match(script_property, property)) {
				indigo_property_copy_values(script_property, property, false);
				script_property->state = INDIGO_OK_STATE;
				if (strcmp(script_property->label, script_property->items[0].text.value)) {
					indigo_delete_property(device, script_property, NULL);
					indigo_copy_value(script_property->label, script_property->items[0].text.value);
					indigo_define_property(device, script_property, NULL);
				} else {
					indigo_update_property(device, script_property, NULL);
				}
				save_config(device);
				return INDIGO_OK;
			}
		}
	}
	return indigo_device_change_property(device, client, property);
}

static indigo_result agent_enable_blob(indigo_device *device, indigo_client *client, indigo_property *property, indigo_enable_blob_mode mode) {
	assert(device != NULL);
	return INDIGO_OK;
}

static indigo_result agent_device_detach(indigo_device *device) {
	assert(device != NULL);
	if (PRIVATE_DATA->ctx) {
		execute_script(AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY);
		duk_destroy_heap(PRIVATE_DATA->ctx);
	}
	pthread_mutex_destroy(&PRIVATE_DATA->mutex);
	indigo_release_property(AGENT_SCRIPTING_ON_LOAD_SCRIPT_PROPERTY);
	indigo_release_property(AGENT_SCRIPTING_ON_UNLOAD_SCRIPT_PROPERTY);
	indigo_release_property(AGENT_SCRIPTING_ADD_SCRIPT_PROPERTY);
	indigo_release_property(AGENT_SCRIPTING_DELETE_SCRIPT_PROPERTY);
	indigo_release_property(AGENT_SCRIPTING_EXECUTE_SCRIPT_PROPERTY);
	for (int i = 0; i < MAX_USER_SCRIPT_COUNT; i++) {
		indigo_property *script_property = AGENT_SCRIPTING_SCRIPT_PROPERTY(i);
		if (script_property)
			indigo_release_property(script_property);
	}
	return indigo_device_detach(device);
}

// -------------------------------------------------------------------------------- INDIGO agent client implementation

static indigo_result agent_client_attach(indigo_client *client) {
	assert(client != NULL);
	indigo_property all_properties;
	memset(&all_properties, 0, sizeof(all_properties));
	indigo_enumerate_properties(client, &all_properties);
	return INDIGO_OK;
}

static void push_state(indigo_property_state state) {
	switch (state) {
		case INDIGO_IDLE_STATE:
			duk_push_string(PRIVATE_DATA->ctx, "Idle");
			break;
		case INDIGO_OK_STATE:
			duk_push_string(PRIVATE_DATA->ctx, "Ok");
			break;
		case INDIGO_BUSY_STATE:
			duk_push_string(PRIVATE_DATA->ctx, "Busy");
			break;
		case INDIGO_ALERT_STATE:
			duk_push_string(PRIVATE_DATA->ctx, "Alert");
			break;
	}
}

static void push_items(indigo_property *property) {
	duk_get_prop_string(PRIVATE_DATA->ctx, -4, "Object");
	if (duk_pnew(PRIVATE_DATA->ctx, 0)) {
		INDIGO_DRIVER_DEBUG(DRIVER_NAME, "Object() failed (%s)", duk_safe_to_string(PRIVATE_DATA->ctx, -1));
	} else {
		for (int i = 0; i < property->count; i++) {
			indigo_item *item = property->items + i;
			switch (property->type) {
				case INDIGO_TEXT_VECTOR:
					duk_push_string(PRIVATE_DATA->ctx, item->text.value);
					break;
				case INDIGO_NUMBER_VECTOR:
					duk_push_number(PRIVATE_DATA->ctx, item->number.value);
					break;
				case INDIGO_SWITCH_VECTOR:
					duk_push_boolean(PRIVATE_DATA->ctx, item->sw.value);
					break;
				case INDIGO_LIGHT_VECTOR:
					push_state(item->light.value);
					break;
				case INDIGO_BLOB_VECTOR:
					duk_push_string(PRIVATE_DATA->ctx, item->blob.url);
					break;
			}
			duk_put_prop_string(PRIVATE_DATA->ctx, -2, item->name);
		}
	}
}

static indigo_result agent_define_property(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	duk_push_global_object(PRIVATE_DATA->ctx);
	if (duk_get_prop_string(PRIVATE_DATA->ctx, -1, "indigo_on_define_property")) {
		duk_push_string(PRIVATE_DATA->ctx, property->device);
		duk_push_string(PRIVATE_DATA->ctx, property->name);
		push_items(property);
		push_state(property->state);
		duk_push_string(PRIVATE_DATA->ctx, property->perm == INDIGO_RW_PERM ? "RW" : property->perm == INDIGO_RO_PERM ? "RO" : "WO");
		duk_push_string(PRIVATE_DATA->ctx, message);
		if (duk_pcall(PRIVATE_DATA->ctx, 6)) {
			INDIGO_DRIVER_DEBUG(DRIVER_NAME, "indigo_on_define_property() call failed (%s)", duk_safe_to_string(PRIVATE_DATA->ctx, -1));
		}
	}
	duk_pop_2(PRIVATE_DATA->ctx);
	return INDIGO_OK;
}

static indigo_result agent_update_property(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	duk_push_global_object(PRIVATE_DATA->ctx);
	if (duk_get_prop_string(PRIVATE_DATA->ctx, -1, "indigo_on_update_property")) {
		duk_push_string(PRIVATE_DATA->ctx, property->device);
		duk_push_string(PRIVATE_DATA->ctx, property->name);
		push_items(property);
		push_state(property->state);
		duk_push_string(PRIVATE_DATA->ctx, message);
		if (duk_pcall(PRIVATE_DATA->ctx, 5)) {
			INDIGO_DRIVER_DEBUG(DRIVER_NAME, "indigo_on_update_property() call failed (%s)", duk_safe_to_string(PRIVATE_DATA->ctx, -1));
		}
	}
	duk_pop_2(PRIVATE_DATA->ctx);
	return INDIGO_OK;
}

static indigo_result agent_delete_property(indigo_client *client, indigo_device *device, indigo_property *property, const char *message) {
	duk_push_global_object(PRIVATE_DATA->ctx);
	if (duk_get_prop_string(PRIVATE_DATA->ctx, -1, "indigo_on_delete_property")) {
		duk_push_string(PRIVATE_DATA->ctx, property->device);
		duk_push_string(PRIVATE_DATA->ctx, property->name);
		duk_push_string(PRIVATE_DATA->ctx, message);
		if (duk_pcall(PRIVATE_DATA->ctx, 3)) {
			INDIGO_DRIVER_DEBUG(DRIVER_NAME, "indigo_on_delete_property() call failed (%s)", duk_safe_to_string(PRIVATE_DATA->ctx, -1));
		}
	}
	duk_pop_2(PRIVATE_DATA->ctx);
	return INDIGO_OK;
}

static indigo_result agent_send_message(indigo_client *client, indigo_device *device, const char *message) {
	duk_push_global_object(PRIVATE_DATA->ctx);
	if (duk_get_prop_string(PRIVATE_DATA->ctx, -1, "agent_on_send_message")) {
		duk_push_string(PRIVATE_DATA->ctx, device->name);
		duk_push_string(PRIVATE_DATA->ctx, message);
		if (duk_pcall(PRIVATE_DATA->ctx, 2)) {
			INDIGO_DRIVER_DEBUG(DRIVER_NAME, "agent_on_send_message() call failed (%s)", duk_safe_to_string(PRIVATE_DATA->ctx, -1));
		}
	}
	duk_pop_2(PRIVATE_DATA->ctx);
	return INDIGO_OK;
}

static indigo_result agent_client_detach(indigo_client *client) {
	return INDIGO_OK;
}

// -------------------------------------------------------------------------------- Initialization

indigo_result indigo_agent_scripting(indigo_driver_action action, indigo_driver_info *info) {
	static indigo_device agent_device_template = INDIGO_DEVICE_INITIALIZER(
		ALIGNMENT_AGENT_NAME,
		agent_device_attach,
		agent_enumerate_properties,
		agent_change_property,
		agent_enable_blob,
		agent_device_detach
	);

	static indigo_client agent_client_template = {
		ALIGNMENT_AGENT_NAME, false, NULL, INDIGO_OK, INDIGO_VERSION_CURRENT, NULL,
		agent_client_attach,
		agent_define_property,
		agent_update_property,
		agent_delete_property,
		agent_send_message,
		agent_client_detach
	};

	static indigo_driver_action last_action = INDIGO_DRIVER_SHUTDOWN;

	SET_DRIVER_INFO(info, ALIGNMENT_AGENT_NAME, __FUNCTION__, DRIVER_VERSION, false, last_action);

	if (action == last_action)
		return INDIGO_OK;

	switch(action) {
		case INDIGO_DRIVER_INIT:
			last_action = action;
			private_data = malloc(sizeof(agent_private_data));
			assert(private_data != NULL);
			memset(private_data, 0, sizeof(agent_private_data));
			agent_device = malloc(sizeof(indigo_device));
			assert(agent_device != NULL);
			memcpy(agent_device, &agent_device_template, sizeof(indigo_device));
			agent_device->private_data = private_data;
			indigo_attach_device(agent_device);

			agent_client = malloc(sizeof(indigo_client));
			assert(agent_client != NULL);
			memcpy(agent_client, &agent_client_template, sizeof(indigo_client));
			agent_client->client_context = agent_device->device_context;
			indigo_attach_client(agent_client);
			break;

		case INDIGO_DRIVER_SHUTDOWN:
			last_action = action;
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
