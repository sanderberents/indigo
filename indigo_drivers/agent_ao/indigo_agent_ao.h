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
 \file indigo_agent_ao.h
 */

#ifndef agent_ao_h
#define agent_ao_h

#include <indigo/indigo_agent.h>

#if defined(INDIGO_WINDOWS)
#if defined(INDIGO_WINDOWS_DLL)
#define INDIGO_EXTERN __declspec(dllexport)
#else
#define INDIGO_EXTERN __declspec(dllimport)
#endif
#else
#define INDIGO_EXTERN extern
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define AO_AGENT_NAME	"AO Agent"

// Process items of AGENT_START_PROCESS
#define AGENT_AO_CALIBRATION_ITEM_NAME					"CALIBRATION"
#define AGENT_AO_BUMP_CALIBRATION_ITEM_NAME			"BUMP_CALIBRATION"
#define AGENT_AO_EXERCISE_ITEM_NAME							"EXERCISE"
#define AGENT_AO_CENTER_ITEM_NAME								"CENTER"
#define AGENT_AO_TRACKING_ITEM_NAME							"TRACKING"

// AO positioning: relative steps (ao_sx) or absolute percent (SBIG AO-7)
#define AGENT_AO_MODE_PROPERTY_NAME							"AGENT_AO_MODE"
#define AGENT_AO_MODE_RELATIVE_ITEM_NAME				"RELATIVE"
#define AGENT_AO_MODE_ABSOLUTE_ITEM_NAME				"ABSOLUTE"

// Whether the slow part of the AO deflection is passed to the mount with guide pulses
#define AGENT_AO_BUMP_MODE_PROPERTY_NAME				"AGENT_AO_BUMP_MODE"
#define AGENT_AO_BUMP_MODE_DIRECT_ITEM_NAME			"DIRECT"
#define AGENT_AO_BUMP_MODE_OFF_ITEM_NAME				"OFF"

// Which declination pulses the direct bump may send
#define AGENT_AO_DEC_MODE_PROPERTY_NAME					"AGENT_AO_DEC_MODE"
#define AGENT_AO_DEC_MODE_BOTH_ITEM_NAME				"BOTH"
#define AGENT_AO_DEC_MODE_NORTH_ITEM_NAME				"NORTH"
#define AGENT_AO_DEC_MODE_SOUTH_ITEM_NAME				"SOUTH"

// Settings, including the calibration results
#define AGENT_AO_SETTINGS_PROPERTY_NAME					"AGENT_AO_SETTINGS"

// Statistics
#define AGENT_AO_STATS_PROPERTY_NAME						"AGENT_AO_STATS"

// Phases reported by AGENT_AO_STATS.PHASE
#define AGENT_AO_PHASE_IDLE											0
#define AGENT_AO_PHASE_TRACKING									1
#define AGENT_AO_PHASE_CALIBRATING							2
#define AGENT_AO_PHASE_EXERCISING								3
#define AGENT_AO_PHASE_BUMP_CALIBRATING					4
#define AGENT_AO_PHASE_CENTERING								5
#define AGENT_AO_PHASE_FAILED										6
#define AGENT_AO_PHASE_DONE											7

/** Create AO agent instance
 */

INDIGO_EXTERN indigo_result indigo_agent_ao(indigo_driver_action action, indigo_driver_info *info);

#ifdef __cplusplus
}
#endif

#endif /* agent_ao_h */
