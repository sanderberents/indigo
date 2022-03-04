// Copyright (c) 2021 Rumen G.Bogdanovski
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
// 2.0 by Rumen G.Bogdanovski <rumen@skyarchive.org>

// Mathematics were derived independantly based on the common spherical geometry principles and the source code is developed based on the math we derived.
// The idea for the three point polar alignment was borowed from N.I.N.A. and some mount hand controllers that provide such functionality.
// I even had some discusson with Stefan Berg on the final touches. He was kind enough to point me to some "gotchas" he faced during the N.I.N.A. development.
// I also want to thank to Dr.Alexander Kurtenkov and Prof.Tanyu Bonev for the disussions we had during the development, and Stoyan Glushkov
// for staying late at hinght and testing the implemntation.

/** INDIGO Bus
 \file indigo_align.h
 */

#ifndef indigo_align_h
#define indigo_align_h

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

/** Cartesian point
 */
typedef struct {
	double x;
	double y;
	double z;
} indigo_cartesian_point_t;

/** Spherical point
 */
typedef struct {
	double a;   /* longitude, az, ra, ha in radians*/
	double d;   /* latitude, alt, dec in radians */
	double r;   /* radius (1 for celestial coordinates) */
} indigo_spherical_point_t;


/**
 Precesses c0 from eq0 to eq1

 c0.a - Right Ascension (radians)
 c0.d - Declination (radians)
 eq0 - Old Equinox (year+fraction)
 eq1 - New Equinox (year+fraction)
 */
indigo_spherical_point_t indigo_precess(const indigo_spherical_point_t *c0, const double eq0, const double eq1);

/** convert ha dec to az alt in radians
 */
extern void indigo_equatorial_to_hotizontal(const indigo_spherical_point_t *eq_point, const double latitude, indigo_spherical_point_t *h_point);

/** convert spherical to cartesian coordinates
 */
extern indigo_cartesian_point_t indigo_spherical_to_cartesian(const indigo_spherical_point_t *spoint);

/** convert spherical (in radians) to cartesian coordinates
 */
extern indigo_spherical_point_t indigo_cartesian_to_sphercal(const indigo_cartesian_point_t *cpoint);

/** rotate cartesian coordinates around axes (angles in radians)
 */
extern indigo_cartesian_point_t indigo_cartesian_rotate_x(const indigo_cartesian_point_t *point, double angle);
extern indigo_cartesian_point_t indigo_cartesian_rotate_y(const indigo_cartesian_point_t *point, double angle);
extern indigo_cartesian_point_t indigo_cartesian_rotate_z(const indigo_cartesian_point_t *point, double angle);

/** rotate coordinates using polar errors
 * possition->a = Hour angle in radians
 * possition->d = Declination in radians
 * possition->r = 1; (should be 1)
 * u = angle in radians, Altitude error.
 * v = angle in radians, Azimuth error.
 */
extern indigo_spherical_point_t indigo_apply_polar_error(const indigo_spherical_point_t *position, double u, double v);

/** derotate coordinates using polar errors
 * possition->a = Hour angle in radians
 * possition->d = Declination in radians
 * possition->r = 1; (should be 1)
 * u = angle in radians, Altitude error.
 * v = angle in radians, Azimuth error.
 */
extern indigo_spherical_point_t indigo_correct_polar_error(const indigo_spherical_point_t *position, double u, double v);


/** convert spherical point in radians to ha/ra dec in hours and degrees
 */
extern void indigo_point_to_ra_dec(const indigo_spherical_point_t *spoint, const double lst, double *ra, double *dec);

/** convert ha/ra dec in hours and degrees to spherical point in radians
 */
extern void indigo_ra_dec_to_point(const double ra, const double dec, const double lst, indigo_spherical_point_t *spoint);

/** great circle distances of sphericaal coordinate poits
 */
extern double indigo_gc_distance_spherical(const indigo_spherical_point_t *sp1, const indigo_spherical_point_t *sp2);

/** great circle distance of ra/ha dec in degrees
 */
extern double indigo_gc_distance(double ra1, double dec1, double ra2, double dec2);

/** great circle distance of points in cartesian coordinates
 */
extern double indigo_gc_distance_cartesian(const indigo_cartesian_point_t *cp1, const indigo_cartesian_point_t *cp2);

/** calculate refraction error from zenith distance
 */
extern double indigo_calculate_refraction(const double z);

/** compensate atmospheric refraction
 */
extern bool indigo_compensate_refraction(
	const indigo_spherical_point_t *st,
	const double latitude,
	indigo_spherical_point_t *st_corrected
);

/** compensate atmospheric refraction
 */
extern bool indigo_compensate_refraction2(
	const indigo_spherical_point_t *st,
	const double latitude,
	const double refraction,
	indigo_spherical_point_t *st_corrected
);

/** calculate polar alignment error and Declination drifts
 */
extern bool indigo_polar_alignment_error_3p(
	const indigo_spherical_point_t *p1,
	const indigo_spherical_point_t *p2,
	const indigo_spherical_point_t *p3,
	double *d2,
	double *d3,
	double *u,
	double *v
);

/** recalculates polar error for a given target position (if thelescope is aligned) and current position
 */
extern bool indigo_reestimate_polar_error(
	const indigo_spherical_point_t *position,
	const indigo_spherical_point_t *target_position,
	double *u,
	double *v
);

/** calculates the position where the telescope sould point if properly polar aligned
 *  for the given position and polar errors
 */
bool indigo_polar_alignment_target_position(
	const indigo_spherical_point_t *position,
	const double u,
	const double v,
	indigo_spherical_point_t *target_position
);

#ifdef __cplusplus
}
#endif

#endif /* indigo_align_h */
