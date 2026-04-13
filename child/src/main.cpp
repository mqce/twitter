#include <Arduino.h>
#include <IRremote.hpp>
#include <esp_system.h>
#include <stdarg.h>
#include <stdio.h>

namespace {
constexpr uint8_t kIrTxPin = 6;
constexpr uint8_t kIrRxPin = 7;
constexpr uint8_t kPiezoPin = 4;

constexpr uint8_t kSelfId = 0x02;
constexpr uint8_t kPartnerId = 0x01;

constexpr uint32_t kHelloGapLimitMs = 250;
constexpr uint8_t kHandshakeSuccessCount = 3;
// Give parent enough time to finish TX and return to RX.
constexpr uint32_t kAckDelayMs = 80;
constexpr uint32_t kHelloLostTimeoutMs = 2000;
constexpr uint32_t kPartnerSignalLostTimeoutMs = 2000;
constexpr uint32_t kPartnerTurnTimeoutMs = 10000;
constexpr uint32_t kActiveIntervalMs = 150;
constexpr uint8_t kDoneRepeatCount = 3;
constexpr uint32_t kTalkingMinDurationMs = 600;
constexpr uint32_t kTalkingMaxDurationMs = 2000;
constexpr uint32_t kTalkStartDelayMinMs = 200;
constexpr uint32_t kTalkStartDelayMaxMs = 800;
constexpr uint16_t kVoiceFreqMinHz = 650;
constexpr uint16_t kVoiceFreqMaxHz = 2400;
constexpr uint32_t kVoiceSyllableMinMs = 90;
constexpr uint32_t kVoiceSyllableMaxMs = 280;
constexpr uint32_t kVoiceGapMinMs = 30;
constexpr uint32_t kVoiceGapMaxMs = 130;
constexpr uint32_t kVoicePhrasePauseMinMs = 120;
constexpr uint32_t kVoicePhrasePauseMaxMs = 320;
constexpr uint8_t kVoicePhraseSyllablesMin = 2;
constexpr uint8_t kVoicePhraseSyllablesMax = 5;
constexpr uint16_t kVoiceSweepMinHz = 140;
constexpr uint16_t kVoiceSweepMaxHz = 780;
constexpr uint16_t kVoiceWarbleDepthMinHz = 10;
constexpr uint16_t kVoiceWarbleDepthMaxHz = 90;
constexpr uint8_t kVoiceWarbleCyclesMin = 1;
constexpr uint8_t kVoiceWarbleCyclesMax = 4;
constexpr uint32_t kVoiceModUpdateIntervalMs = 45;
constexpr uint32_t kVoiceLeadInMinMs = 500;
constexpr uint32_t kVoiceLeadInMaxMs = 1000;

enum class MsgType : uint8_t {
  Hello = 0x01,
  Ack = 0x02,
  ActiveParent = 0x11,
  DoneParent = 0x12,
  ActiveChild = 0x21,
  DoneChild = 0x22,
};

enum class ConversationState : uint8_t {
  Idle,
  ParentTalking,
  ChildTalking,
};

struct Frame {
  MsgType type;
  uint8_t srcId;
  uint8_t dstId;
};

enum class VoiceContour : uint8_t {
  Rise,
  Fall,
  Arch,
  Dip,
};

uint8_t consecutiveHelloCount = 0;
uint32_t lastHelloAtMs = 0;
uint32_t scheduledAckAtMs = 0;
uint32_t lastPartnerSignalAtMs = 0;
uint32_t stateEnteredAtMs = 0;
uint32_t partnerTurnStartedAtMs = 0;
uint32_t lastActiveAtMs = 0;
uint32_t currentTurnEndsAtMs = 0;
uint32_t lastDoneSentAtMs = 0;
uint8_t pendingDoneRepeats = 0;
bool handshakeComplete = false;
ConversationState state = ConversationState::Idle;
bool voiceToneOn = false;
uint32_t voiceSegmentStartedAtMs = 0;
uint32_t voiceSegmentEndsAtMs = 0;
uint32_t voiceRestEndsAtMs = 0;
uint32_t voiceNextUpdateAtMs = 0;
uint32_t voiceLeadInEndsAtMs = 0;
uint16_t voiceStartFreqHz = 0;
uint16_t voiceMidFreqHz = 0;
uint16_t voiceEndFreqHz = 0;
uint16_t voiceWarbleDepthHz = 0;
uint8_t voiceWarbleCycles = 0;
uint8_t voicePhraseRemaining = 0;
VoiceContour voiceContour = VoiceContour::Rise;
bool childTalkStartPending = false;
uint32_t childTalkStartAtMs = 0;
const char *childTalkStartReason = nullptr;

const char *stateName(const ConversationState value) {
  switch (value) {
    case ConversationState::Idle:
      return "IDLE";
    case ConversationState::ParentTalking:
      return "PARENT_TALKING";
    case ConversationState::ChildTalking:
      return "CHILD_TALKING";
  }
  return "UNKNOWN";
}

const char *currentState() { return stateName(state); }

void logStatef(const char *fmt, ...) {
  char message[196];
  va_list args;
  va_start(args, fmt);
  vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);
  Serial.printf("[CHILD][%s] %s\n", currentState(), message);
}

void changeState(const ConversationState nextState, const char *reason) {
  if (state == nextState) {
    return;
  }

  logStatef("STATE CHANGED: %s -> %s (%s)", stateName(state), stateName(nextState), reason);
  state = nextState;
  stateEnteredAtMs = millis();
}

void resetConversationState(const char *reason) {
  const ConversationState previousState = state;
  consecutiveHelloCount = 0;
  scheduledAckAtMs = 0;
  handshakeComplete = false;
  lastPartnerSignalAtMs = 0;
  stateEnteredAtMs = 0;
  partnerTurnStartedAtMs = 0;
  lastActiveAtMs = 0;
  currentTurnEndsAtMs = 0;
  lastDoneSentAtMs = 0;
  pendingDoneRepeats = 0;
  voiceToneOn = false;
  voiceSegmentStartedAtMs = 0;
  voiceSegmentEndsAtMs = 0;
  voiceRestEndsAtMs = 0;
  voiceNextUpdateAtMs = 0;
  voiceLeadInEndsAtMs = 0;
  voicePhraseRemaining = 0;
  childTalkStartPending = false;
  childTalkStartAtMs = 0;
  childTalkStartReason = nullptr;
  noTone(kPiezoPin);
  state = ConversationState::Idle;
  if (previousState != ConversationState::Idle) {
    Serial.printf("[CHILD][%s] STATE CHANGED: %s -> %s (%s)\n",
                  stateName(previousState),
                  stateName(previousState),
                  stateName(ConversationState::Idle),
                  reason);
  }
}

void sendFrame(MsgType type) {
  const uint16_t address = (static_cast<uint16_t>(kSelfId) << 8) | kPartnerId;
  IrSender.sendNEC(address, static_cast<uint8_t>(type), 0);
}

bool isAcceptedProtocol(const decode_type_t protocol) {
  return protocol == NEC || protocol == NEC2;
}

bool decodePartnerFrame(Frame &frame) {
  if (!IrReceiver.decode()) {
    return false;
  }

  const auto decoded = IrReceiver.decodedIRData;
  IrReceiver.resume();

  if (!isAcceptedProtocol(decoded.protocol)) {
    return false;
  }

  const uint8_t src = static_cast<uint8_t>((decoded.address >> 8) & 0xFF);
  const uint8_t dst = static_cast<uint8_t>(decoded.address & 0xFF);
  if (src == kSelfId && dst == kPartnerId) {
    return false;
  }

  if (src != kPartnerId || dst != kSelfId) {
    return false;
  }

  switch (decoded.command) {
    case static_cast<uint8_t>(MsgType::Hello):
      frame.type = MsgType::Hello;
      break;
    case static_cast<uint8_t>(MsgType::ActiveParent):
      frame.type = MsgType::ActiveParent;
      break;
    case static_cast<uint8_t>(MsgType::DoneParent):
      frame.type = MsgType::DoneParent;
      break;
    default:
      return false;
  }

  frame.srcId = src;
  frame.dstId = dst;
  return true;
}

uint32_t nextTalkingDurationMs() {
  return static_cast<uint32_t>(random(kTalkingMinDurationMs, kTalkingMaxDurationMs + 1));
}

uint32_t randomRangeMs(const uint32_t minValue, const uint32_t maxValue) {
  return static_cast<uint32_t>(random(minValue, maxValue + 1));
}

uint16_t randomRangeHz(const uint16_t minValue, const uint16_t maxValue) {
  return static_cast<uint16_t>(random(minValue, maxValue + 1));
}

uint16_t clampVoiceFreqHz(const int32_t value) {
  return static_cast<uint16_t>(
      constrain(value, static_cast<int32_t>(kVoiceFreqMinHz), static_cast<int32_t>(kVoiceFreqMaxHz)));
}

uint16_t lerpHz(const uint16_t startValue, const uint16_t endValue, const uint32_t progress256) {
  const int32_t delta = static_cast<int32_t>(endValue) - static_cast<int32_t>(startValue);
  return clampVoiceFreqHz(static_cast<int32_t>(startValue) +
                          ((delta * static_cast<int32_t>(progress256)) / 256));
}

void stopVoiceSynth() {
  if (voiceToneOn) {
    noTone(kPiezoPin);
    voiceToneOn = false;
  }
  voiceSegmentStartedAtMs = 0;
  voiceSegmentEndsAtMs = 0;
  voiceRestEndsAtMs = 0;
  voiceNextUpdateAtMs = 0;
  voiceLeadInEndsAtMs = 0;
  voicePhraseRemaining = 0;
}

uint16_t currentVoiceFreqHz(const uint32_t now) {
  if (voiceSegmentEndsAtMs <= voiceSegmentStartedAtMs) {
    return voiceMidFreqHz;
  }

  const uint32_t durationMs = voiceSegmentEndsAtMs - voiceSegmentStartedAtMs;
  const uint32_t elapsedMs = now <= voiceSegmentStartedAtMs ? 0 : now - voiceSegmentStartedAtMs;
  const uint32_t progress256 = min<uint32_t>(256, (elapsedMs * 256UL) / durationMs);

  uint16_t contourFreq = voiceMidFreqHz;
  if (progress256 < 128) {
    contourFreq = lerpHz(voiceStartFreqHz, voiceMidFreqHz, progress256 * 2);
  } else {
    contourFreq = lerpHz(voiceMidFreqHz, voiceEndFreqHz, (progress256 - 128) * 2);
  }

  if (voiceWarbleDepthHz == 0 || voiceWarbleCycles == 0) {
    return contourFreq;
  }

  const uint32_t phase = (elapsedMs * static_cast<uint32_t>(voiceWarbleCycles) * 512UL) / durationMs;
  int32_t triangle = static_cast<int32_t>(phase % 512UL);
  if (triangle > 255) {
    triangle = 511 - triangle;
  }
  const int32_t centered = triangle - 128;
  return clampVoiceFreqHz(static_cast<int32_t>(contourFreq) +
                          (centered * static_cast<int32_t>(voiceWarbleDepthHz)) / 128);
}

void startVoiceSyllable(const uint32_t now) {
  if (voicePhraseRemaining == 0) {
    voicePhraseRemaining = static_cast<uint8_t>(
        random(kVoicePhraseSyllablesMin, kVoicePhraseSyllablesMax + 1));
  }

  const uint16_t anchorFreqHz = randomRangeHz(kVoiceFreqMinHz, kVoiceFreqMaxHz);
  const uint16_t sweepHz = randomRangeHz(kVoiceSweepMinHz, kVoiceSweepMaxHz);
  voiceContour = static_cast<VoiceContour>(random(4));

  switch (voiceContour) {
    case VoiceContour::Rise:
      voiceStartFreqHz = clampVoiceFreqHz(anchorFreqHz - (sweepHz / 2));
      voiceMidFreqHz = clampVoiceFreqHz(anchorFreqHz + (sweepHz / 3));
      voiceEndFreqHz = clampVoiceFreqHz(anchorFreqHz + sweepHz);
      break;
    case VoiceContour::Fall:
      voiceStartFreqHz = clampVoiceFreqHz(anchorFreqHz + sweepHz);
      voiceMidFreqHz = clampVoiceFreqHz(anchorFreqHz + (sweepHz / 3));
      voiceEndFreqHz = clampVoiceFreqHz(anchorFreqHz - (sweepHz / 2));
      break;
    case VoiceContour::Arch:
      voiceStartFreqHz = clampVoiceFreqHz(anchorFreqHz - (sweepHz / 3));
      voiceMidFreqHz = clampVoiceFreqHz(anchorFreqHz + sweepHz);
      voiceEndFreqHz = clampVoiceFreqHz(anchorFreqHz - (sweepHz / 2));
      break;
    case VoiceContour::Dip:
      voiceStartFreqHz = clampVoiceFreqHz(anchorFreqHz + (sweepHz / 2));
      voiceMidFreqHz = clampVoiceFreqHz(anchorFreqHz - sweepHz);
      voiceEndFreqHz = clampVoiceFreqHz(anchorFreqHz + (sweepHz / 4));
      break;
  }

  voiceWarbleDepthHz = randomRangeHz(kVoiceWarbleDepthMinHz, kVoiceWarbleDepthMaxHz);
  voiceWarbleCycles = static_cast<uint8_t>(
      random(kVoiceWarbleCyclesMin, kVoiceWarbleCyclesMax + 1));
  voiceSegmentStartedAtMs = now;
  voiceSegmentEndsAtMs = now + randomRangeMs(kVoiceSyllableMinMs, kVoiceSyllableMaxMs);
  voiceRestEndsAtMs = 0;
  voiceNextUpdateAtMs = 0;
  voiceToneOn = true;
  --voicePhraseRemaining;
}

void updateVoiceSynth(const uint32_t now, const bool localTalking) {
  if (!localTalking) {
    stopVoiceSynth();
    return;
  }

  if (now < voiceLeadInEndsAtMs) {
    if (voiceToneOn) {
      noTone(kPiezoPin);
      voiceToneOn = false;
    }
    return;
  }

  if (voiceToneOn) {
    if (now >= voiceSegmentEndsAtMs) {
      noTone(kPiezoPin);
      voiceToneOn = false;
      voiceNextUpdateAtMs = 0;
      voiceRestEndsAtMs = now + (voicePhraseRemaining > 0
                                     ? randomRangeMs(kVoiceGapMinMs, kVoiceGapMaxMs)
                                     : randomRangeMs(kVoicePhrasePauseMinMs, kVoicePhrasePauseMaxMs));
      return;
    }

    if (voiceNextUpdateAtMs == 0 || now >= voiceNextUpdateAtMs) {
      tone(kPiezoPin, currentVoiceFreqHz(now));
      voiceNextUpdateAtMs = now + kVoiceModUpdateIntervalMs;
    }
    return;
  }

  if (voiceRestEndsAtMs != 0 && now < voiceRestEndsAtMs) {
    return;
  }

  startVoiceSyllable(now);
}

void startChildTalking(const char *reason) {
  handshakeComplete = true;
  childTalkStartPending = false;
  childTalkStartAtMs = 0;
  childTalkStartReason = nullptr;
  partnerTurnStartedAtMs = 0;
  lastPartnerSignalAtMs = 0;
  lastDoneSentAtMs = 0;
  pendingDoneRepeats = 0;
  lastActiveAtMs = 0;
  currentTurnEndsAtMs = millis() + nextTalkingDurationMs();
  voiceLeadInEndsAtMs = millis() + randomRangeMs(kVoiceLeadInMinMs, kVoiceLeadInMaxMs);
  changeState(ConversationState::ChildTalking, reason);
  logStatef("Simulating child speech pattern for %lu ms",
            static_cast<unsigned long>(currentTurnEndsAtMs - millis()));
}

void scheduleChildTalking(const uint32_t now, const char *reason) {
  if (state == ConversationState::ChildTalking || childTalkStartPending) {
    return;
  }
  childTalkStartPending = true;
  childTalkStartReason = reason;
  childTalkStartAtMs = now + randomRangeMs(kTalkStartDelayMinMs, kTalkStartDelayMaxMs);
}

void maybeStartScheduledChildTalking(const uint32_t now) {
  if (!childTalkStartPending || now < childTalkStartAtMs) {
    return;
  }
  childTalkStartPending = false;
  childTalkStartAtMs = 0;
  startChildTalking(childTalkStartReason != nullptr ? childTalkStartReason
                                                    : "scheduled start");
  childTalkStartReason = nullptr;
}

void finishChildTalking() {
  logStatef("Child finished speech pattern; handing baton to parent");
  sendFrame(MsgType::DoneChild);
  lastDoneSentAtMs = millis();
  pendingDoneRepeats = kDoneRepeatCount > 0 ? kDoneRepeatCount - 1 : 0;
  currentTurnEndsAtMs = 0;
  lastActiveAtMs = 0;
  lastPartnerSignalAtMs = 0;
  partnerTurnStartedAtMs = 0;
  changeState(ConversationState::ParentTalking, "child turn complete");
}

void updateChildTalking(const uint32_t now) {
  if (lastActiveAtMs == 0 || now - lastActiveAtMs >= kActiveIntervalMs) {
    sendFrame(MsgType::ActiveChild);
    lastActiveAtMs = now;
  }

  if (currentTurnEndsAtMs != 0 && now >= currentTurnEndsAtMs) {
    finishChildTalking();
  }
}

void updateParentTalking(const uint32_t now) {
  if (partnerTurnStartedAtMs == 0) {
    // Once we hand off to the parent, lack of any ACTIVE,P means the session
    // is gone. Reset quickly so a fresh HELLO/ACK handshake can restart.
    if (stateEnteredAtMs != 0 && now - stateEnteredAtMs > kPartnerSignalLostTimeoutMs) {
      resetConversationState("parent turn start timeout");
    }
    return;
  }

  if (now - lastPartnerSignalAtMs > kPartnerSignalLostTimeoutMs) {
    resetConversationState("parent signal lost");
    return;
  }

  if (now - partnerTurnStartedAtMs > kPartnerTurnTimeoutMs) {
    resetConversationState("parent DONE timeout");
  }
}

void keepSendingDoneChildWhileWaiting(const uint32_t now) {
  if (state != ConversationState::ParentTalking || partnerTurnStartedAtMs != 0) {
    return;
  }

  if (pendingDoneRepeats > 0 && now - lastDoneSentAtMs >= kActiveIntervalMs) {
    sendFrame(MsgType::DoneChild);
    lastDoneSentAtMs = now;
    --pendingDoneRepeats;
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  randomSeed(static_cast<uint32_t>(esp_random()));
  pinMode(kPiezoPin, OUTPUT);
  noTone(kPiezoPin);

  IrSender.begin(kIrTxPin, DISABLE_LED_FEEDBACK);
  IrReceiver.begin(kIrRxPin, DISABLE_LED_FEEDBACK);
}

void loop() {
  const uint32_t now = millis();
  Frame frame{};
  if (decodePartnerFrame(frame)) {
    switch (frame.type) {
      case MsgType::Hello:
        if (state != ConversationState::Idle) {
          // Parent only sends HELLO while idle. Seeing HELLO mid-session means
          // both sides lost sync, so drop back to handshake immediately.
          resetConversationState("received HELLO while not idle");
        }

        if (lastHelloAtMs != 0 && now - lastHelloAtMs <= kHelloGapLimitMs) {
          ++consecutiveHelloCount;
        } else {
          consecutiveHelloCount = 1;
        }

        lastHelloAtMs = now;
        scheduledAckAtMs = now + kAckDelayMs;
        if (!handshakeComplete) {
          logStatef("RX HELLO (%u/%u)", consecutiveHelloCount, kHandshakeSuccessCount);
        }

        if (!handshakeComplete && consecutiveHelloCount >= kHandshakeSuccessCount) {
          handshakeComplete = true;
          logStatef("Handshake complete; waiting for parent turn");
        }
        break;

      case MsgType::ActiveParent:
        handshakeComplete = true;
        if (state != ConversationState::ParentTalking) {
          changeState(ConversationState::ParentTalking, "received ACTIVE,P");
        }
        if (partnerTurnStartedAtMs == 0) {
          partnerTurnStartedAtMs = now;
          logStatef("Parent turn started");
        }
        lastPartnerSignalAtMs = now;
        break;

      case MsgType::DoneParent:
        handshakeComplete = true;
        lastPartnerSignalAtMs = now;
        if (state != ConversationState::ChildTalking) {
          startChildTalking("received DONE,P");
        }
        break;

      default:
        break;
    }
  }

  if (scheduledAckAtMs != 0 && now >= scheduledAckAtMs) {
    sendFrame(MsgType::Ack);
    scheduledAckAtMs = 0;
  }

  if (state == ConversationState::Idle && handshakeComplete && lastHelloAtMs != 0 &&
      now - lastHelloAtMs > kHelloLostTimeoutMs) {
    logStatef("Handshake reset (HELLO timeout)");
    handshakeComplete = false;
    consecutiveHelloCount = 0;
    lastHelloAtMs = 0;
  }

  if (state == ConversationState::ChildTalking) {
    updateChildTalking(now);
  } else if (state == ConversationState::ParentTalking) {
    updateParentTalking(now);
    keepSendingDoneChildWhileWaiting(now);
  }
  maybeStartScheduledChildTalking(now);

  updateVoiceSynth(now, state == ConversationState::ChildTalking);
}
