#include "RustyBulletCapturePlugin.h"

#include "bakkesmod/wrappers/arraywrapper.h"
#include "bakkesmod/wrappers/GameObject/CarComponent/BoostWrapper.h"

#include <sstream>

// `PLUGINTYPE` (bakkesmodsdk.h) has no flag for a regular local/offline
// match at all -- only FREEPLAY, CUSTOM_TRAINING, SPECTATOR, BOTAI, REPLAY,
// THREADED, THREADEDUNLOAD exist. `PLUGINTYPE_FREEPLAY` is this plugin's
// primary use case (see README); it doesn't gate loading during a normal
// match, since there's no bit for one to begin with.
BAKKESMOD_PLUGIN(RustyBulletCapturePlugin, "Rusty Bullet capture", "1.1", PLUGINTYPE_FREEPLAY)

namespace
{
// Field names/nesting below must match crates/rb_capture_ingest/src/wire.rs
// exactly (see ADR-0005) -- this is the wire format itself, not incidental
// formatting.

std::string vectorJson(Vector v)
{
    std::ostringstream out;
    out << "{\"x\":" << v.X << ",\"y\":" << v.Y << ",\"z\":" << v.Z << "}";
    return out.str();
}

std::string quatJson(Quat q)
{
    std::ostringstream out;
    out << "{\"x\":" << q.X << ",\"y\":" << q.Y << ",\"z\":" << q.Z << ",\"w\":" << q.W << "}";
    return out.str();
}

std::string rbActorJson(RBActorWrapper actor)
{
    RBState state = actor.GetRBState();
    std::ostringstream out;
    out << "{\"position\":" << vectorJson(state.Location) << ",\"rotation\":" << quatJson(state.Quaternion)
        << ",\"velocity\":" << vectorJson(state.LinearVelocity)
        << ",\"angular_velocity\":" << vectorJson(state.AngularVelocity) << "}";
    return out.str();
}

// `rb_domain::ControllerInput::{throttle,steer}` are plain `f32`, and
// `pitch`/`yaw`/`roll` are always `Some` for a capture (see ADR-0005 and
// `CarState.input`'s doc comment) -- unlike a replay, a live capture always
// has the analog stick values, so no field here is ever omitted.
std::string inputJson(ControllerInput input)
{
    std::ostringstream out;
    out << "{\"throttle\":" << input.Throttle << ",\"steer\":" << input.Steer << ",\"pitch\":" << input.Pitch
        << ",\"yaw\":" << input.Yaw << ",\"roll\":" << input.Roll << ",\"jump\":" << (input.Jump ? "true" : "false")
        << ",\"boost\":" << (input.HoldingBoost ? "true" : "false")
        << ",\"handbrake\":" << (input.Handbrake ? "true" : "false") << "}";
    return out.str();
}

// `rb_domain::CarState::boost_amount` is 0-100 (see
// `rb_replay_ingest::convert::boost_raw_to_percent`'s doc comment) --
// `BoostWrapper::GetCurrentBoostAmount()` is the 0.0-1.0 fraction, so scale
// it here to keep both ingestion adapters in the same unit.
// Everything of a car's line except its `input` and the closing brace --
// the input arrives separately, from this car's own `SetVehicleInput`
// firing (see `onVehicleInput`).
std::string carPrefixJson(int playerId, CarWrapper car)
{
    std::ostringstream out;
    out << "{\"player_id\":" << playerId << ",";
    std::string actor = rbActorJson(car);
    // rbActorJson() returns a full `{...}` object; splice its fields in
    // rather than nesting it, since CarState's wire shape is flat plus
    // `boost_amount`/`input`, not `{"actor": {...}, ...}`.
    out << actor.substr(1, actor.size() - 2);
    out << ",\"boost_amount\":" << (car.GetBoostComponent().GetCurrentBoostAmount() * 100.0f);
    return out.str();
}

const char *const kNeutralInputJson =
    "{\"throttle\":0,\"steer\":0,\"pitch\":0,\"yaw\":0,\"roll\":0,\"jump\":false,\"boost\":false,\"handbrake\":false}";
} // namespace

void RustyBulletCapturePlugin::onLoad()
{
    gameWrapper->HookEventWithCallerPost<CarWrapper>(
        "Function TAGame.Car_TA.SetVehicleInput",
        [this](CarWrapper car, void *params, std::string eventName) { onVehicleInput(car, params, eventName); });

    cvarManager->registerNotifier(
        "rb_capture_start",
        [this](std::vector<std::string> args) { startCapture(args); },
        "Start recording a Rusty Bullet capture file: rb_capture_start <path.jsonl>",
        PERMISSION_ALL);

    cvarManager->registerNotifier(
        "rb_capture_stop",
        [this](std::vector<std::string> args) { stopCapture(args); },
        "Stop the current Rusty Bullet capture recording, if any",
        PERMISSION_ALL);
}

void RustyBulletCapturePlugin::onUnload()
{
    stopCapture({});
}

void RustyBulletCapturePlugin::startCapture(std::vector<std::string> args)
{
    std::string path = args.size() > 1 ? args[1] : "rusty_bullet_capture.jsonl";

    if (captureFile_.is_open())
    {
        captureFile_.close();
    }

    captureFile_.open(path, std::ios::out | std::ios::trunc);
    if (!captureFile_.is_open())
    {
        cvarManager->log("rusty_bullet_capture: failed to open '" + path + "' for writing");
        return;
    }

    capturing_ = true;
    lastPhysicsFrame_ = -1;
    haveStartTime_ = false;
    havePending_ = false;
    lastInputJson_.clear();
    cvarManager->log("rusty_bullet_capture: recording to '" + path + "'");
}

void RustyBulletCapturePlugin::stopCapture(std::vector<std::string> /*args*/)
{
    if (!capturing_)
    {
        return;
    }

    flushPending();
    capturing_ = false;
    captureFile_.close();
    cvarManager->log("rusty_bullet_capture: stopped recording");
}

void RustyBulletCapturePlugin::onVehicleInput(CarWrapper car, void *params, std::string /*eventName*/)
{
    if (!capturing_ || car.IsNull())
    {
        return;
    }

    ServerWrapper server = gameWrapper->GetCurrentGameState();
    if (server.IsNull())
    {
        return;
    }

    BallWrapper ball = server.GetBall();
    if (ball.IsNull())
    {
        return;
    }

    // `SetVehicleInput` fires once per car per physics tick; every car's
    // firing this tick sees the same `GetPhysicsFrame()`. The first firing
    // of a new frame closes the previous frame's line and snapshots this
    // frame's state; the inputs are filled in firing by firing below, so a
    // match with N cars still produces exactly one line per tick.
    int physicsFrame = ball.GetPhysicsFrame();
    if (physicsFrame != lastPhysicsFrame_)
    {
        lastPhysicsFrame_ = physicsFrame;
        flushPending();
        beginFrame(server, ball);
    }

    // The input is the struct this very call was made with -- `params` is
    // `SetVehicleInput`'s one argument, a `ControllerInput` -- recorded
    // against this car. Version 1.0 instead read every car's
    // `CarWrapper::GetInput()` back at the *first* firing of the tick,
    // which is only fresh if that car's own `SetVehicleInput` has already
    // run this tick; whether it had depended on firing order, and
    // `RB-PHYSICS-001-FR-085` (finding I) found the result in real
    // captures: whole clips with every analog axis recorded as `0`, a
    // dodge whose `jump` press was never recorded, a pitch input arriving
    // one tick after the flip it caused.
    if (params == nullptr)
    {
        return;
    }
    ControllerInput input = *static_cast<ControllerInput *>(params);
    std::string json = inputJson(input);
    std::uintptr_t key = car.memory_address;
    lastInputJson_[key] = json;
    if (havePending_)
    {
        for (PendingCar &pendingCar : pending_.cars)
        {
            if (pendingCar.key == key)
            {
                pendingCar.input = json;
                pendingCar.haveInput = true;
            }
        }
    }
}

void RustyBulletCapturePlugin::flushPending()
{
    if (!havePending_)
    {
        return;
    }
    havePending_ = false;

    std::ostringstream line;
    line << pending_.header;
    bool first = true;
    for (const PendingCar &pendingCar : pending_.cars)
    {
        if (!first)
        {
            line << ",";
        }
        first = false;
        line << pendingCar.prefix << ",\"input\":";
        if (pendingCar.haveInput)
        {
            line << pendingCar.input;
        }
        else
        {
            auto last = lastInputJson_.find(pendingCar.key);
            line << (last != lastInputJson_.end() ? last->second : std::string(kNeutralInputJson));
        }
        line << "}";
    }
    line << "]}";

    captureFile_ << line.str() << "\n";
    captureFile_.flush();
}

void RustyBulletCapturePlugin::beginFrame(ServerWrapper server, BallWrapper ball)
{
    if (server.IsNull() || ball.IsNull())
    {
        return;
    }

    if (!haveStartTime_)
    {
        // First frame of this recording: treat its own physics time as
        // t=0, matching `PhysicsFrame::timestamp_secs`'s doc comment
        // ("seconds since the start of the capture/replay, not a
        // wall-clock time").
        startPhysicsTime_ = ball.GetPhysicsTime();
        haveStartTime_ = true;
    }
    float timestampSecs = ball.GetPhysicsTime() - startPhysicsTime_;

    // `server.GetPRIs()` + `PriWrapper::GetCar()` looked like the natural way
    // to enumerate cars, but a real capture proved it wrong: in freeplay the
    // PRI's `Car` back-reference never gets updated to the live-driven pawn
    // (PRI exists for scoreboard/stat tracking, which freeplay has none of),
    // so every line recorded the same frozen spawn-point transform with
    // all-zero input while the ball moved for real. `GameEventWrapper::GetCars()`
    // (inherited via `ServerWrapper` -> `TeamGameEventWrapper`) is the game's
    // own live list of spawned car actors -- the same source cameras/
    // scoreboards use -- and reflects real movement.
    ArrayWrapper<CarWrapper> cars = server.GetCars();

    std::ostringstream header;
    header << "{\"timestamp_secs\":" << timestampSecs << ",\"ball\":" << rbActorJson(ball) << ",\"cars\":[";
    pending_.header = header.str();
    pending_.cars.clear();

    int nextPlayerId = 0;
    // `player_id` here is just this recording session's car iteration order,
    // not a stable cross-session id -- BakkesMod's own PRI/unique-id
    // wrappers exist, but a one-off capture script (see RB-VERIFY-002's
    // Non-goals) never replays a capture against a second session, so a
    // per-session ordinal is all `rb_capture_ingest` needs.
    if (!cars.IsNull())
    {
        for (int i = 0; i < cars.Count(); ++i)
        {
            CarWrapper car = cars.Get(i);
            if (car.IsNull())
            {
                continue;
            }

            PendingCar pendingCar;
            pendingCar.key = car.memory_address;
            pendingCar.prefix = carPrefixJson(nextPlayerId, car);
            pendingCar.haveInput = false;
            pending_.cars.push_back(pendingCar);
            ++nextPlayerId;
        }
    }

    havePending_ = true;
}
