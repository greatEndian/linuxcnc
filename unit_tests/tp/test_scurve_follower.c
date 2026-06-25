/* test_scurve_follower.c — unit coverage for the jerk-limited S-curve velocity
 * FOLLOWER algorithm (tpCalculateSCurveFollowerAccel in tp.c).
 *
 * The follower has no automated coverage via the integration tests (none of
 * them use PLANNER_TYPE=1, and the motion-logger replaces motmod so the TP
 * never runs). Here we exercise the follower's core math directly, with the
 * real shipped sp_scurve primitives (nextSpeed / findSCurveMaxStartSpeed /
 * stoppingDist), by simulating the exact per-cycle loop the follower runs:
 *
 *     dx      = remaining distance
 *     v_decel = findSCurveMaxStartSpeed(dx, v_final, A, J)   // closed-form
 *     v_goal  = min(v_cruise, v_decel)
 *     nextSpeed(v, a, dt, v_goal, A, J, &v, &a, &j)          // jerk-limited step
 *     progress += (v_prev + v) * dt / 2                       // trapezoidal
 *
 * and asserting the safety invariants that "smooth at the cusps" does not
 * prove: the segment is fully traversed, the corner is entered at no more than
 * finalvel, and the accel / jerk / velocity limits are honoured every cycle.
 */
#include "greatest.h"
#include "sp_scurve.h"
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include "rtapi.h"

/* Expand greatest's definitions into this translation unit. */
GREATEST_MAIN_DEFS();

/* sp_scurve.c (via libtp) references rtapi_print_msg; provide a stub so the
 * test links standalone, matching test_blendmath.c. */
void rtapi_print_msg(msg_level_t level, const char *fmt, ...)
{
    va_list args;
    (void)level;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

typedef struct {
    double v_final;   /* velocity when the segment end was reached */
    double max_v;     /* peak |velocity| seen                       */
    double max_a;     /* peak |acceleration| seen                   */
    double max_j;     /* peak |jerk| seen                           */
    double progress;  /* distance travelled                         */
    long   iters;     /* cycles taken                               */
    int    reached;   /* hit the segment end within the iter cap    */
} sim_result_t;

/* Simulate the follower loop over one segment of length L, cruising at Vcruise,
 * arriving at Vfinal, under accel/jerk limits A/J at timestep dt. Mirrors
 * tpCalculateSCurveFollowerAccel + tcUpdateDistFromSCurveAccel (trapezoidal). */
static sim_result_t simulate_follower(double L, double Vcruise, double Vfinal,
                                      double A, double J, double dt)
{
    sim_result_t r = {0.0, 0.0, 0.0, 0.0, 0.0, 0, 0};
    double v = 0.0, a = 0.0;
    const long maxiter = 5000000;
    long i;

    for (i = 0; i < maxiter; i++) {
        double dx = L - r.progress;
        if (dx <= 1e-9) { r.reached = 1; break; }

        double v_decel = Vcruise;
        if (findSCurveMaxStartSpeed(dx, Vfinal, A, J, &v_decel) != 1) {
            v_decel = Vcruise;
        }
        double v_goal = fmin(Vcruise, v_decel);

        double nv = v, na = a, nj = 0.0;
        nextSpeed(v, a, dt, v_goal, A, J, &nv, &na, &nj);

        double disp = (v + nv) * dt / 2.0;
        r.progress += disp;
        v = nv;
        a = na;

        if (fabs(v)  > r.max_v) r.max_v = fabs(v);
        if (fabs(a)  > r.max_a) r.max_a = fabs(a);
        if (fabs(nj) > r.max_j) r.max_j = fabs(nj);

        /* Near the end with essentially-zero residual speed (Vfinal==0): the
         * trapezoidal step can only crawl, so treat a tiny remaining gap as
         * reached rather than spinning to the iter cap. */
        if (dx < 1e-6 && v < 1e-4) { r.progress = L; r.reached = 1; break; }
        if (r.progress >= L - 1e-9) { r.reached = 1; break; }
    }
    r.v_final = v;
    r.iters = i;
    return r;
}

/* Integrate a jerk-limited deceleration from v0 (a=0) down to 0 using the same
 * nextSpeed primitive the follower uses, and return the distance covered. This
 * is the ground-truth the follower actually experiences (not stoppingDist,
 * which models a different, ~2x quantity and is not used by the follower). */
static double decel_distance(double v0, double A, double J, double dt)
{
    double v = v0, a = 0.0, d = 0.0;
    for (int i = 0; i < 5000000 && v > 1e-7; i++) {
        double nv, na, nj;
        nextSpeed(v, a, dt, 0.0, A, J, &nv, &na, &nj);
        d += (v + nv) * dt / 2.0;
        v = nv; a = na;
    }
    return d;
}

/* findSCurveMaxStartSpeed (the follower's decel limit) must be sane: succeed,
 * grow monotonically with distance, and -- the key SAFETY invariant -- a free
 * jerk-limited deceleration from the returned speed must come to rest WITHIN
 * the gap (never overrun it).
 *
 * Note on what this does and does NOT show: decel_distance() here drives
 * nextSpeed straight to a target of 0 (a "panic stop"), which is more
 * aggressive than the curve the follower actually rides, so it covers only
 * ~half the gap. That is just safety margin -- the follower can always brake
 * harder than required. It is NOT slack in cornering speed: in real operation
 * the follower TRACKS v_decel = findSCurveMaxStartSpeed(dx, finalvel) down as
 * dx shrinks, so it cruises at full speed until exactly the true jerk-limited
 * decel point and then rides the optimal curve to finalvel (measured
 * onset/theory ratio = 1.0). */
TEST decel_limit_is_sane_and_safe(void)
{
    const double A = 500.0, J = 5000.0, dt = 0.0005;
    double prev_vs = -1.0;
    for (double D = 0.25; D <= 64.0; D *= 2.0) {
        double vs = 0.0;
        ASSERT_EQ(1, findSCurveMaxStartSpeed(D, 0.0, A, J, &vs));
        ASSERT(vs > 0.0);
        ASSERT(vs > prev_vs);            /* monotonic in distance */
        prev_vs = vs;

        double d = decel_distance(vs, A, J, dt);
        /* SAFETY: must stop within the gap (small discrete margin allowed). */
        ASSERTm("follower would overrun the decel gap", d <= D * 1.02 + 1e-4);
        ASSERT(d > 0.0);
    }
    PASS();
}

/* nextSpeed must honour the accel/jerk limits every step and converge to the
 * target. It may transiently overshoot the target velocity by a small amount
 * (discrete jerk-limited approach), so bound that rather than forbid it. */
TEST nextSpeed_respects_limits_and_converges(void)
{
    const double A = 500.0, J = 5000.0, dt = 0.001, targetV = 80.0;
    double v = 0.0, a = 0.0, max_overshoot = 0.0;
    int converged = 0;
    for (int i = 0; i < 20000; i++) {
        double nv, na, nj;
        nextSpeed(v, a, dt, targetV, A, J, &nv, &na, &nj);
        ASSERT(fabs(na) <= A + 1e-6);          /* accel limit */
        ASSERT(fabs(nj) <= J + 1e-6);          /* jerk limit  */
        if (nv - targetV > max_overshoot) max_overshoot = nv - targetV;
        v = nv; a = na;
        if (fabs(v - targetV) < 1e-6 && fabs(a) < 1e-6) { converged = 1; break; }
    }
    ASSERT(converged);
    ASSERT_IN_RANGE(targetV, v, 1e-3);
    /* overshoot must be small (well under 1% of target) */
    ASSERTm("velocity overshoot too large", max_overshoot <= 0.01 * targetV);
    PASS();
}

/* End-to-end: the follower must traverse the segment, arrive at no more than
 * finalvel (so it never overruns the corner), and never break the limits. */
TEST follower_traverses_segment_within_limits(void)
{
    const double A = 500.0, J = 5000.0, dt = 0.001;
    struct { double L, V, Vf; } cases[] = {
        {10.0,  50.0,  0.0},   /* stop-to-stop                  */
        {20.0,  80.0, 20.0},   /* through-corner (nonzero final) */
        { 2.0, 100.0,  0.0},   /* short: decel-limited from start*/
        {50.0, 200.0, 50.0},   /* long cruise + decel            */
        { 0.5, 100.0,  0.0},   /* very short stop                */
    };
    for (unsigned c = 0; c < sizeof(cases)/sizeof(cases[0]); c++) {
        double L = cases[c].L, V = cases[c].V, Vf = cases[c].Vf;
        sim_result_t r = simulate_follower(L, V, Vf, A, J, dt);

        ASSERTm("segment not fully traversed", r.reached);
        ASSERTm("acceleration limit violated",  r.max_a <= A * 1.01);
        ASSERTm("jerk limit violated",          r.max_j <= J * 1.01);
        ASSERTm("velocity limit violated",      r.max_v <= V * 1.01);

        /* Must enter the corner at <= finalvel (key safety: no overrun). */
        ASSERTm("overran finalvel at corner", r.v_final <= Vf + fmax(0.02 * V, 3.0 * A * dt));
        ASSERT(r.v_final >= -1e-6);
        /* And actually arrive near finalvel, not stall far short. */
        ASSERT_IN_RANGE(Vf, r.v_final, fmax(0.05 * V, 5.0 * A * dt));
    }
    PASS();
}

/* The follower must corner OPTIMALLY, not conservatively: cruise at full speed
 * until exactly the true jerk-limited deceleration point, then ride the curve
 * down. Catches regressions that would make it brake too early (slow cornering)
 * or too late (overrun). */
TEST follower_corners_at_optimal_point(void)
{
    const double A = 500.0, J = 5000.0, dt = 0.0005;
    const double L = 300.0, V = 100.0, Vf = 0.0;   /* long: cruise then decel */
    double v = 0.0, a = 0.0, prog = 0.0, onset_dx = -1.0, v_at_onset = -1.0;

    for (long i = 0; i < 50000000 && prog < L - 1e-9; i++) {
        double dx = L - prog;
        double vdec = V;
        findSCurveMaxStartSpeed(dx, Vf, A, J, &vdec);
        double vgoal = fmin(V, vdec);
        if (onset_dx < 0.0 && vdec < V - 1e-6) { onset_dx = dx; v_at_onset = v; }
        double nv, na, nj;
        nextSpeed(v, a, dt, vgoal, A, J, &nv, &na, &nj);
        prog += (v + nv) * dt / 2.0;
        v = nv; a = na;
        if (dx < 1e-6 && v < 1e-4) break;
    }

    /* true jerk-limited decel distance V->Vf (invert findSCurveMaxStartSpeed) */
    double dlo = 0.0, dhi = 400.0;
    for (int k = 0; k < 60; k++) {
        double dm = (dlo + dhi) / 2.0, vv;
        findSCurveMaxStartSpeed(dm, Vf, A, J, &vv);
        if (vv < V) dlo = dm; else dhi = dm;
    }
    double D = (dlo + dhi) / 2.0;

    ASSERT(onset_dx > 0.0);
    ASSERTm("did not cruise at full speed before decel", fabs(v_at_onset - V) <= 1e-2);
    ASSERTm("decel did not begin at the optimal point",  fabs(onset_dx - D) <= 0.02 * D);
    ASSERT_IN_RANGE(L, prog, 1e-3);   /* lands exactly on the endpoint */
    PASS();
}

SUITE(scurve_follower) {
    RUN_TEST(decel_limit_is_sane_and_safe);
    RUN_TEST(nextSpeed_respects_limits_and_converges);
    RUN_TEST(follower_traverses_segment_within_limits);
    RUN_TEST(follower_corners_at_optimal_point);
}

int main(int argc, char **argv)
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(scurve_follower);
    GREATEST_MAIN_END();
}
