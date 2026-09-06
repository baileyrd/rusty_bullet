#pragma once

#include "bakkesmod/plugin/bakkesmodplugin.h"
#include "bakkesmod/plugin/bakkesmodsdk.h"
#include "bakkesmod/wrappers/GameWrapper.h"
#include "bakkesmod/wrappers/GameEvent/ServerWrapper.h"
#include "bakkesmod/wrappers/GameObject/BallWrapper.h"
#include "bakkesmod/wrappers/GameObject/CarWrapper.h"

#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <vector>

// Records a JSON-Lines capture file matching ADR-0005 / RB-VERIFY-002-FR-001:
// one line per physics tick, `{"timestamp_secs", "ball", "cars"}`, readable
// by `rb_capture_ingest` without any changes on the Rust side. Built,
// loaded, and run against a real Rocket League + BakkesMod install -- see
// this directory's README.md and RB-VERIFY-002's spec Change history.
class RustyBulletCapturePlugin : public BakkesMod::Plugin::BakkesModPlugin
{
public:
    void onLoad() override;
    void onUnload() override;

private:
    // Hooked once per car per physics tick (`Function TAGame.Car_TA.SetVehicleInput`,
    // post). The first firing of a new physics frame flushes the previous
    // frame's line and snapshots this frame's ball/car state
    // (`beginFrame`); every firing then records *its own* car's input --
    // the `ControllerInput` the hook hands over in `params`, not
    // `CarWrapper::GetInput()` read back at some other car's firing (see
    // `onVehicleInput` for the bug that read produced). The line is
    // written once the next frame begins, so it carries the inputs every
    // car actually received for that tick.
    void onVehicleInput(CarWrapper car, void *params, std::string eventName);

    void startCapture(std::vector<std::string> args);
    void stopCapture(std::vector<std::string> args);

    // Snapshots the ball's and every car's state for a new physics frame
    // into `pending_`, inputs still to come. Does nothing if either wrapper
    // is null.
    void beginFrame(ServerWrapper server, BallWrapper ball);

    // Appends `pending_`'s line to the capture file, if there is one. A car
    // whose `SetVehicleInput` never fired during the frame gets its last
    // recorded input (or a neutral one, if it never fired at all).
    void flushPending();

    struct PendingCar
    {
        std::uintptr_t key;
        std::string prefix; // `{"player_id":..,<state>,"boost_amount":..` -- no closing brace
        std::string input;  // `inputJson(...)` once this car's hook fired this frame
        bool haveInput;
    };

    struct PendingFrame
    {
        std::string header; // `{"timestamp_secs":..,"ball":{..},"cars":[`
        std::vector<PendingCar> cars;
    };

    std::ofstream captureFile_;
    bool capturing_ = false;
    int lastPhysicsFrame_ = -1;
    bool haveStartTime_ = false;
    float startPhysicsTime_ = 0.0f;
    bool havePending_ = false;
    PendingFrame pending_;
    std::map<std::uintptr_t, std::string> lastInputJson_;
};
