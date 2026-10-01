// SYNTHETIC host harness: formats records with the app's real QuestXarmTelemetry.h and passes
// them through the SAME emission gate (prepare_log_message) used before __android_log_write().
// Not an Android build: it does not exercise Android log delivery.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "QuestXarmTelemetry.h"
using namespace questxarm;

static PoseSample tracked(float x, float y, float z, int active) {
    PoseSample s; s.queried = true; s.result = 0; s.action_active = active;
    s.flags = kPositionValid | kOrientationValid | kPositionTracked | kOrientationTracked;
    s.p[0] = x; s.p[1] = y; s.p[2] = z;
    s.q[0] = 0; s.q[1] = 0.3826834f; s.q[2] = 0; s.q[3] = 0.9238795f;  // 45 deg about +Y
    return s;
}

static void emit(const std::string& json, const char* build, const char* sid, uint64_t seq) {
    const std::string msg = prepare_log_message(json, build, sid, seq);
    fwrite(msg.data(), 1, msg.size(), stdout);
    fputc('\n', stdout);
}

int main(int argc, char** argv) {
    const char* sc = argc > 1 ? argv[1] : "full";
    const char* sid = "00000000deadbeef";
    if (!strcmp(sc, "probe")) {  // probe <id> <bytes> <utf8 0/1>
        const int id = atoi(argv[2]);
        const size_t n = strtoul(argv[3], nullptr, 10);
        emit(make_probe("s4a-test", sid, 100 + id, id, n, atoi(argv[4]) != 0), "s4a-test", sid, 100 + id);
        return 0;
    }
    if (!strcmp(sc, "event_space")) {
        Event e; e.build = "s4a-test"; e.session_id = sid; e.seq = 7;
        e.name = "local_space_change_pending"; e.time_ns = 5000; e.local_space_epoch = 1; e.pose_valid = 1;
        e.previous = tracked(0.1f, 0, 0, -1);
        emit(format_event_record(e), e.build, sid, e.seq);
        return 0;
    }
    std::string longbuild;
    Frame f; f.build = "s4a-test"; f.session_id = sid; f.seq = 8; f.t_query_ns = 123456789012LL;
    f.session_state = "FOCUSED"; f.focused = true;
    f.head = tracked(0, 1.6f, 0, -1);
    f.grip_l = tracked(-0.2f, 1.0f, -0.3f, 1);
    f.grip_r = tracked(0.2f, 1.0f, -0.3f, 1);
    f.in_r.trigger = {true, 0.75f}; f.in_r.squeeze = {true, 0.3f};
    f.in_r.button_lower = {true, true}; f.in_r.button_upper = {true, false};
    f.in_r.thumbstick = {true, 0.0f, -0.5f};
    f.in_l.trigger = {true, 0.0f};
    if (!strcmp(sc, "worst")) {  // every field active with long-form numbers: worst realistic size
        for (PoseSample* s : {&f.head, &f.grip_l, &f.grip_r}) {
            s->result = -2147483647; s->p[0] = -12345.123456f; s->p[1] = -12345.123456f; s->p[2] = -12345.123456f;
            s->q[0] = -0.5f; s->q[1] = -0.5f; s->q[2] = -0.5f; s->q[3] = -0.5f;
        }
        f.seq = 18446744073709551615ULL; f.t_query_ns = -9223372036854775807LL; f.local_space_epoch = 4294967295u;
        f.last_local_change_time_ns = -9223372036854775807LL; f.session_state = "LOSS_PENDING";
        for (HandInputs* h : {&f.in_l, &f.in_r}) {
            h->trigger = {true, -1.0f}; h->squeeze = {true, -1.0f}; h->thumbstick = {true, -1.0f, -1.0f};
            h->thumbstick_click = h->thumbstick_touch = h->trigger_touch = h->thumbrest_touch = {true, false};
            h->button_lower = h->button_upper = h->menu = {true, false};
        }
    }
    if (!strcmp(sc, "right_missing")) { f.grip_r = PoseSample(); f.grip_r.queried = true; f.grip_r.action_active = 0; f.in_r = HandInputs(); }
    if (!strcmp(sc, "untracked")) { f.grip_r.flags = kPositionValid | kOrientationValid; }
    if (!strcmp(sc, "nonfinite")) { f.grip_r.p[1] = NAN; }
    if (!strcmp(sc, "unfocused")) { f.session_state = "VISIBLE"; f.focused = false; f.grip_l = PoseSample(); f.grip_l.queried = true; f.grip_l.action_active = 0;
                                    f.grip_r = f.grip_l; f.in_l = HandInputs(); f.in_r = HandInputs(); }
    if (!strcmp(sc, "overflow")) { longbuild.assign(3000, 'B'); f.build = longbuild.c_str(); }
    // Identity for the overflow record is the app's short build id (the long build string only
    // serves to push the SYNTHETIC record over the cap).
    emit(format_pose_record(f), "s4a-test", sid, f.seq);
    return 0;
}
