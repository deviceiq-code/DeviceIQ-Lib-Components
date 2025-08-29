#include "DevIQ_Components.h"

using namespace DeviceIQ_Components;

Generic::Generic(String name, int16_t id) : mName(name), mID(id) {
    Event.insert({
        {"Changed",[&](callback_t callback) { mChanged = callback; }}
    });
}

Generic::Generic(String name, int16_t id, Buses bus, uint8_t address) : mName(name), mID(id), mBus(bus), mAddress(address) {
    Event.insert({
        {"Changed",[&](callback_t callback) { mChanged = callback; }}
    });
}

Relay::Relay(String name, int16_t id, Buses bus, uint8_t address, RelayTypes type) : Generic(name, id, bus, address), mType(type) {
    Event.insert({
        {"SettingOn",[&](callback_t callback) { mSettingOn = callback; }},
        {"SettingOff",[&](callback_t callback) { mSettingOff = callback; }},
        {"SetOn",[&](callback_t callback) { mSetOn = callback; }},
        {"SetOff",[&](callback_t callback) { mSetOff = callback; }},
    });

    switch (mBus) {
        case BUS_I2C: {
            pcf8574 = new PCF8574(mAddress);
            pcf8574->begin();
        } break;
        default: {
            pinMode(mAddress, OUTPUT);
        } break;
    }
}

const void Relay::State(bool newstate, bool savestate) {
    if (!mEnabled) return;

    mState = (mType == RELAYTYPE_NORMALLYOPENED ? !newstate : newstate);

    if (newstate) { if (mSettingOn) mSettingOn(); } else { if (mSettingOff) mSettingOff(); }

    switch (mBus) {
        case BUS_I2C: { pcf8574->digitalWrite(mAddress, mState); } break;
        default: { digitalWrite(mAddress, mState); } break;
    }

    if (newstate) { if (mSetOn) mSetOn(); } else { if (mSetOff) mSetOff(); }
    if (mChanged) mChanged();
}

PIR::PIR(String name, int16_t id, Buses bus, uint8_t address): Generic(name, id, bus, address), mState(false), mPrevState(false) {
    Event.insert({
        {"MotionDetected", [&](callback_t callback) { mMotionDetected = callback; }},
        {"MotionCleared", [&](callback_t callback) { mMotionCleared = callback; }}
    });

    switch (mBus) {
        case BUS_I2C:
            // If there is an I2C version of PIR, would initialize here
            break;
        default:
            pinMode(mAddress, INPUT);
            break;
    }

    mState = digitalRead(mAddress);
    mPrevState = mState;
}

void PIR::Control() {
    if (!mEnabled) return;

    bool currentState = digitalRead(mAddress);
    uint32_t now = millis();

    if (currentState != mPrevState && (now - mLastChangeMs) >= mDebounceTimeMs) {
        mPrevState = currentState;
        mLastChangeMs = now;
        mState = currentState;

        if (mChanged) mChanged();
        if (mState && mMotionDetected) mMotionDetected();
        if (!mState && mMotionCleared) mMotionCleared();
    }
}


Button::Button(String name, int16_t id, Buses bus, uint8_t address, ButtonReportModes reportmode) : Generic(name, id, bus, address), mReportMode(reportmode) {
    switch(reportmode) {
        case ButtonReportModes::BUTTONREPORTMODE_EDGESONLY: {
             Event.insert({
                {"Pressed",[&](callback_t callback) { mPressed = callback; }},
                {"Released",[&](callback_t callback) { mReleased = callback; }}
            });
        } break;
        case ButtonReportModes::BUTTONREPORTMODE_CLICKSONLY: {
             Event.insert({
                {"Clicked",[&](callback_t callback) { mClicked = callback; }},
                {"DoubleClicked",[&](callback_t callback) { mDoubleClicked = callback; }},
                {"TripleClicked",[&](callback_t callback) { mTripleClicked = callback; }},
                {"LongClicked",[&](callback_t callback) { mLongClicked = callback; }}
            });
        } break;
    }

    switch (mBus) {
        case BUS_I2C: {
            pcf8574 = new PCF8574(mAddress);
            pcf8574->begin();
        } break;
        default: {
            pinMode(mAddress, INPUT_PULLUP);
        } break;
    }

    mState = digitalRead(mAddress);
    mPrev_State = mState;
}

void Button::Control() {
    if (!mEnabled) return;

    mPrev_State = mState;
    uint32_t now = millis();

    switch (mBus) {
        case BUS_I2C: { 
            mState = pcf8574->digitalRead(mAddress);
        } break;
        default: {
            mState = digitalRead(mAddress);
        } break;
    }

    if (mState == mPressedState && mPrev_State != mPressedState) {
        mDown_Ms = now;
        mPressed_Triggered = false;
        mClick_Ms = mDown_Ms;

        if (mReportMode == ButtonReportModes::BUTTONREPORTMODE_EDGESONLY) {
            mSequenceSuppressClicks = true;
        } else {
            mSequenceSuppressClicks = false;
        }
    } else if (mState == mPressedState && !mPressed_Triggered && (now - mDown_Ms >= Debounce_Time_Ms)) {
        mPressed_Triggered = true;

        if (mReportMode == ButtonReportModes::BUTTONREPORTMODE_EDGESONLY) {
            if (mChanged) mChanged();
            if (mPressed) mPressed();
            mSequenceSuppressClicks = true;
        } else {
            mClick_Count++;
            if (mChanged) mChanged();
            if (mPressed) { }
        }
    } else if (mState != mPressedState && mPrev_State == mPressedState) {
        mDown_Time_Ms = now - mDown_Ms;
        if (mDown_Time_Ms >= Debounce_Time_Ms) {
            if (mChanged) mChanged();

            if (mReportMode == ButtonReportModes::BUTTONREPORTMODE_EDGESONLY) {
                if (mReleased) mReleased();
                mSequenceSuppressClicks = true;
                mClick_Count = 0;
                mLongClick_Detected = false;
                mLongClick_Detected_Reported = false;
                mLongClick_Detected_Counter = 0;
            } else {
                if (mDown_Time_Ms >= LongClick_Time_Ms) mLongClick_Detected = true;
            }
        }
    } else if (mState != mPressedState && now - mClick_Ms > DoubleClick_Time_Ms) {

        if (!mSequenceSuppressClicks) {
            if (mLongClick_Detected) {
                if (mClick_Count == 1) { mLast_Click_Type = CLICKTYPE_LONG; if (mLongClicked) mLongClicked(); }
                mLongClick_Detected = false;
                mLongClick_Detected_Reported = false;
                mLongClick_Detected_Counter = 0;
            } else if (mClick_Count > 0) {
                switch (mClick_Count) {
                    case 1: { mLast_Click_Type = CLICKTYPE_SINGLE; if (mClicked)       mClicked();       } break;
                    case 2: { mLast_Click_Type = CLICKTYPE_DOUBLE; if (mDoubleClicked)  mDoubleClicked(); } break;
                    case 3: { mLast_Click_Type = CLICKTYPE_TRIPLE; if (mTripleClicked)  mTripleClicked(); } break;
                }
            }
        }
        mClick_Count = 0;
        mClick_Ms = 0;
    }

    bool longclick_period_detected = now - mDown_Ms >= (LongClick_Time_Ms * (mLongClick_Detected_Counter + 1));
    if (mState == mPressedState && longclick_period_detected && !mLongClick_Detected_Reported) {
        if (mReportMode == ButtonReportModes::BUTTONREPORTMODE_CLICKSONLY && !mSequenceSuppressClicks) {
            mLongClick_Detected_Reported = true;
            mLongClick_Detected = true;
            if (LongClick_Detected_Retriggerable) { mLongClick_Detected_Counter++; mLongClick_Detected_Reported = false; }
            if (mLongClicked) mLongClicked();
        } else {
            mLongClick_Detected_Reported = true;
        }
    }
}

Blinds::Blinds(String name, int16_t id, Relay* relayup, Relay* relaydown) : Generic(name, id, BUS_GROUP, 0), mRelayUp(relayup), mRelayDown(relaydown) {
    Event.insert({
        {"Closed",[&](callback_t callback) { mClosed = callback; }},
        {"Opened",[&](callback_t callback) { mOpened = callback; }},
        {"BeforeClose",[&](callback_t callback) { mBeforeClose = callback; }},
        {"BeforeOpen",[&](callback_t callback) { mBeforeOpen = callback; }}
    });

    mTimerUp = new DeviceIQ_DateTime::Timer(mStepMs);
    mTimerDown = new DeviceIQ_DateTime::Timer(mStepMs);

    mTimerUp->OnTimeout([&]{
        if (mCurrentPosition == mTargetPosition) {
            mTimerUp->Stop();
            mRelayUp->State(false);
            mRelayDown->State(false);
            mState = BLINDSSTATE_STOPPED;
        } else {
            if (mCurrentPosition < mTargetPosition) {
                mCurrentPosition++;
                mRelayUp->State(true, false);
                mRelayDown->State(false, false);
            }
            if ((mCurrentPosition == 100) && (mOpened)) {
                mOpened();
                if (mChanged) mChanged();
            }
        }
    });

    mTimerDown->OnTimeout([&]{
        if (mCurrentPosition == mTargetPosition) {
            mTimerDown->Stop();
            mRelayUp->State(false);
            mRelayDown->State(false);
            mState = BLINDSSTATE_STOPPED;
        } else {
            if (mCurrentPosition > mTargetPosition) {
                mCurrentPosition--;
                mRelayUp->State(false, false);
                mRelayDown->State(true, false);
            }
            if ((mCurrentPosition == 0) && (mClosed)) {
                mClosed();
                if (mChanged) mChanged();
            }
        }
    });
}

void Blinds::Position(uint8_t value, bool setformerposition) {
    if (!mEnabled) return;
    
    value = constrain(value, 0, 100);

    if (setformerposition) {
        mCurrentPosition = value;
        mTargetPosition = value;
    } else {
        if (mState != BLINDSSTATE_STOPPED) {
            mRelayUp->State(false, false);
            mRelayDown->State(false, false);
            mTimerUp->Stop();
            mTimerDown->Stop();
        }

        if (value > mCurrentPosition) {
            if (mBeforeOpen != nullptr) mBeforeOpen();
            if (mCancel) {
                mCancel = false;
            } else {
                mRelayDown->State(false, false);
                mTargetPosition = value;
                mTimerUp->Start();
                mState = BLINDSSTATE_INCREASING;
            }
        } else if (value < mCurrentPosition) {
            if (mBeforeClose != nullptr) mBeforeClose();
            if (mCancel) {
                mCancel = false;
            } else {
                mRelayUp->State(false, false);
                mTargetPosition = value;
                mTimerDown->Start();
                mState = BLINDSSTATE_DECREASING;
            }
        }
    }
}

void Blinds::Control() {
    if (!mEnabled) return;
    
    mTimerUp->Control();
    mTimerDown->Control();
}

Thermometer::Thermometer(String name, int16_t id, Buses bus, uint8_t address, ThermometerTypes type, uint32_t autorefreshms) : Generic(name, id, bus, address), mType(type), mAutoRefreshMs(autorefreshms) {
    Event.insert({
        {"TemperatureChanged",[&](callback_t callback) { mTemperatureChanged = callback; }},
        {"HumidityChanged",[&](callback_t callback) { mHumidityChanged = callback; }}
    });

    Updater_Timer = new DeviceIQ_DateTime::Timer(500);
    AutoRefresh_Timer = new DeviceIQ_DateTime::Timer(mAutoRefreshMs);

    switch (mType) {
        case THERMOMETERTYPE_DHT11:
        case THERMOMETERTYPE_DHT12:
        case THERMOMETERTYPE_DHT21:
        case THERMOMETERTYPE_DHT22: {
            dht = new DHT_Unified(mAddress, mType);
            dht->begin();
            Updater_Timer->OnTimeout([&] {
                dht->temperature().getEvent(&dht_event);
                newTemperature = (isnan(dht_event.temperature) ? -127 : dht_event.temperature);
                dht->humidity().getEvent(&dht_event);
                newHumidity = (isnan(dht_event.relative_humidity) ? 0 : dht_event.relative_humidity);
                if (Scale == TEMPERATURESCALE_FAHRENHEIT) newTemperature = (newTemperature * 9.0f / 5.0f) + 32.0f;
                if (fabsf(newTemperature - mTemperature) >= mTemperatureThreshold) { mTemperature = newTemperature; if (mTemperatureChanged) mTemperatureChanged(); }
                if (newHumidity != mHumidity) { mHumidity = newHumidity; if (mHumidityChanged) mHumidityChanged(); if (mChanged) mChanged(); }
            });
        } break;
        case THERMOMETERTYPE_DS18B20: {
            onewire = new OneWire(mAddress);
            dallastemperature = new DallasTemperature(onewire);
            dallastemperature->begin();
            Updater_Timer->OnTimeout([&] {
                dallastemperature->requestTemperatures();
                newTemperature = dallastemperature->getTempCByIndex(0);
                newHumidity = 0;
                if (Scale == TEMPERATURESCALE_FAHRENHEIT) newTemperature = (newTemperature * 9.0f / 5.0f) + 32.0f;
                if (fabsf(newTemperature - mTemperature) >= mTemperatureThreshold) { mTemperature = newTemperature; if (mTemperatureChanged) mTemperatureChanged(); }
                if (newHumidity != mHumidity) { mHumidity = newHumidity; if (mHumidityChanged) mHumidityChanged(); if (mChanged) mChanged(); }
            });
        } break;
    }

    AutoRefresh_Timer->OnTimeout([&] { Refresh(); });

    Updater_Timer->Start();
    if (mAutoRefreshMs > 0) AutoRefresh_Timer->Start();
}

Currentmeter::Currentmeter(String name, int16_t id, Buses bus, uint8_t address) : Generic(name, id, bus, address) {
    Event.insert({
        {"CurrentACChanged",[&](callback_t callback) { mCurrentACChanged = callback; }},
        {"CurrentDCChanged",[&](callback_t callback) { mCurrentDCChanged = callback; }}
    });

    #if defined(ESP32)
    analogSetPinAttenuation(mAddress, ADC_11db);
    #endif

    Updater = new DeviceIQ_DateTime::Timer(1000);

    Updater->OnTimeout([&]{
        if (!mAutoCalibrated) CalibrateZero(2000);

        const uint32_t tStart = millis();
        uint32_t n = 0;
        double sumDiff = 0.0;
        double sumSqDiff = 0.0;

        while ((millis() - tStart) < mWindowMs || n < mMinSamples) {
            int mv = analogReadMilliVolts(mAddress);
            int diff = mv - mZeroOffsetmV;
            sumDiff += diff;
            sumSqDiff += (double)diff * (double)diff;
            n++;
            delayMicroseconds(150);
        }

        double meanDiff = (n > 0) ? (sumDiff / (double)n) : 0.0;
        double rmsDiff = (n > 0) ? sqrt(sumSqDiff / (double)n) : 0.0;

        newCurrentDC = (float)(meanDiff / (double)mMvPerAmp);
        newCurrentAC = (float)(rmsDiff / (double)mMvPerAmp);

        float prevAC = mCurrentAC;
        float prevDC = mCurrentDC;

        mCurrentAC = (1.0f - mAlpha) * mCurrentAC + mAlpha * newCurrentAC;
        mCurrentDC = (1.0f - mAlpha) * mCurrentDC + mAlpha * newCurrentDC;

        if (fabsf(mCurrentAC - prevAC) >= mThresholdAC) { if (mCurrentACChanged) mCurrentACChanged(); }
        if (fabsf(mCurrentDC - prevDC) >= mThresholdDC) { if (mCurrentDCChanged) mCurrentDCChanged(); }
    });

    Updater->Start();
}

void Currentmeter::CalibrateZero(uint16_t samples) {
    uint32_t n = 0;
    int64_t acc = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        acc += (int)analogReadMilliVolts(mAddress);
        n++;
        delayMicroseconds(150);
    }
    if (n > 0) mZeroOffsetmV = (int)(acc / (int64_t)n);
    mAutoCalibrated = true;
}

Doorbell::Doorbell(String name, int16_t id, Buses bus, uint8_t address) : Button(name, id, bus, address, ButtonReportModes::BUTTONREPORTMODE_CLICKSONLY) {
    Event.insert({
        {"Ring", [&](callback_t callback) { mRing = callback; }},
        {"DoubleRing", [&](callback_t callback) { mDoubleRing = callback; }},
        {"LongRing", [&](callback_t callback) { mLongRing = callback; }},
        {"BrightnessChanged", [&](callback_t callback) { mBrightnessChanged = callback; }},
        {"VolumeChanged", [&](callback_t callback) { mVolumeChanged = callback; }}
    });

    Event["Clicked"]([&] {
        mState = 1;
        mLastEventMs = millis();
        if (mRing) mRing();
    });

    Event["DoubleClicked"]([&] {
        mState = 2;
        mLastEventMs = millis();
        if (mDoubleRing) mDoubleRing();
    });

    Event["LongClicked"]([&] {
        mState = 3;
        mLastEventMs = millis();
        if (mLongRing) mLongRing();
    });
}

ContactSensor::ContactSensor(String name, int16_t id, Buses bus, uint8_t address, bool invertClosed) : Button(name, id, bus, address, ButtonReportModes::BUTTONREPORTMODE_EDGESONLY), mInvertClosed(invertClosed) {
    Event.insert({
        {"Opened", [&](callback_t callback) { mOpened = callback; }},
        {"Closed", [&](callback_t callback) { mClosed  = callback; }}
    });

    Event["Pressed"]([&] {
        if (!mEnabled) return;

        if (mInvertClosed) {
            if (mOpened) mOpened();
        } else {
            if (mClosed) mClosed();
        }
        if (mChanged) mChanged();
    });

    Event["Released"]([&] {
        if (!mEnabled) return;

        if (mInvertClosed) {
            if (mClosed) mClosed();
        } else {
            if (mOpened) mOpened();
        }
        if (mChanged) mChanged();
    });

    Event["Changed"]([&] {
        if (!mEnabled) return;
        
        if (mChanged) mChanged();
    });
}

bool Collection::Add(Generic* new_component) {
    if (IndexOf(new_component->Name()) == -1) {
        bool exists = false;
        for (const auto m : mCollection) {
            switch (m->Class()) {
                case CLASS_GENERIC:
                case CLASS_BLINDS: break;
                default: { if ((m->Bus() == new_component->Bus()) && (m->Address() == new_component->Address())) exists = true; } break;
            }
        }
        if (!exists) { mCollection.push_back(new_component); return true; }
    }
    return false;
}