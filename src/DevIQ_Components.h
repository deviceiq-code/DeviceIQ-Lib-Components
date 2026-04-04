#ifndef DevIQ_Components_h
#define DevIQ_Components_h

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <DHT_U.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <PCF8574.h>
#include <map>
#include <vector>
#include <DevIQ_DateTime.h>

namespace DeviceIQ_Components {
    typedef std::function<void()> callback_t;

    enum RunState {
        Running,
        Paused
    };

    enum Classes { 
        CLASS_GENERIC,
        CLASS_RELAY,
        CLASS_PIR,
        CLASS_BUTTON,
        CLASS_BLINDS,
        CLASS_THERMOMETER,
        CLASS_CURRENTMETER,
        CLASS_DOORBELL,
        CLASS_CONTACTSENSOR
    };

    enum Buses { 
        BUS_GROUP,
        BUS_ONBOARD,
        BUS_I2C
    };

    enum RelayTypes { 
        RELAYTYPE_NORMALLYCLOSED, 
        RELAYTYPE_NORMALLYOPENED
    };

    enum class ButtonReportModes {
        BUTTONREPORTMODE_CLICKSONLY,
        BUTTONREPORTMODE_EDGESONLY
    };

    enum ClickTypes { 
        CLICKTYPE_NOCLICK,
        CLICKTYPE_SINGLE,
        CLICKTYPE_DOUBLE,
        CLICKTYPE_TRIPLE,
        CLICKTYPE_LONG
    };

    enum BlindsStates { 
        BLINDSSTATE_STOPPED,
        BLINDSSTATE_INCREASING,
        BLINDSSTATE_DECREASING
    };

    enum TemperatureScales { 
        TEMPERATURESCALE_CELSIUS = 'C',
        TEMPERATURESCALE_FAHRENHEIT = 'F'
    };

    enum ThermometerTypes { 
        THERMOMETERTYPE_DHT11 = 11,
        THERMOMETERTYPE_DHT12 = 12,
        THERMOMETERTYPE_DHT21 = 21,
        THERMOMETERTYPE_DHT22 = 22,
        THERMOMETERTYPE_DS18B20 = 0
    };

    static const std::map<String, Classes> AvailableComponentClasses = {
        {"Generic", CLASS_GENERIC},
        {"Relay", CLASS_RELAY},
        {"PIR", CLASS_PIR},
        {"Button", CLASS_BUTTON},
        {"Blinds", CLASS_BLINDS},
        {"Thermometer", CLASS_THERMOMETER},
        {"Currentmeter", CLASS_CURRENTMETER},
        {"Doorbell", CLASS_DOORBELL},
        {"ContactSensor", CLASS_CONTACTSENSOR}
    };

    static const std::map<String, Buses> AvailableComponentBuses = {
        {"Group", BUS_GROUP},
        {"Onboard", BUS_ONBOARD},
        {"I2C", BUS_I2C}
    };

    static const std::map<String, ThermometerTypes> AvailableThermometerTypes = {
        {"DHT11", THERMOMETERTYPE_DHT11},
        {"DHT12", THERMOMETERTYPE_DHT12},
        {"DHT21", THERMOMETERTYPE_DHT21},
        {"DHT22", THERMOMETERTYPE_DHT22},
        {"DS18B20", THERMOMETERTYPE_DS18B20}
    };

    template<typename T>
    inline static String EnumToString(const std::map<String, T>& table, T value) {
        for (const auto& it : table) if (it.second == value) return it.first;
        return "Unknown";
    }

    class Generic {
        protected:
            String mName;
            int16_t mID;
            bool mEnabled = true;
            Buses mBus = BUS_ONBOARD;
            uint8_t mAddress = 0;
            callback_t mChanged;
        public:
            inline Generic(String name, int16_t id);
            inline Generic(String name, int16_t id, Buses bus, uint8_t address);
            virtual ~Generic() {}

            inline void Name(String value) { mName = value; }
            inline const String& Name() const { return mName; }
            inline int16_t ID() { return mID; }
            inline bool Enabled() { return mEnabled; }
            inline void Enabled(bool value) { mEnabled = value; }
            inline Buses Bus() const { return mBus; }
            inline uint8_t Address() const { return mAddress; }
            virtual inline Classes Class() const { return CLASS_GENERIC; }
            std::map<String, std::function<void(callback_t)>> Event;
            virtual void Control() {}
            inline void Refresh() { if (!mEnabled) return; if(mChanged) mChanged(); }
            virtual bool IsVirtual() const { return false; }

            template <typename T>
            T* as() { return static_cast<T*>(this); }

            virtual callback_t GetEventCallback(const String& eventName);
            virtual bool SetEventCallback(const String& eventName, callback_t cb);
    };

    class Relay : public Generic {
        private:
            bool mState;
            RelayTypes mType;
            PCF8574* pcf8574;
            callback_t mSettingOn, mSettingOff, mSetOn, mSetOff;
        public:
            Relay(String name, int16_t id, Buses bus, uint8_t address, RelayTypes type);
            virtual ~Relay() {}

            inline Classes Class() const override { return CLASS_RELAY; }
            inline const void Invert() { State(!State()); }
            inline const bool State() { return (mType == RELAYTYPE_NORMALLYOPENED ? !mState : mState); }
            const void State(bool newstate, bool savestate = true);
            inline bool operator==(Relay& rhs) { return (this == &rhs); }

            virtual callback_t GetEventCallback(const String& eventName) override;
            virtual bool SetEventCallback(const String& eventName, callback_t cb) override;
    };

    class PIR : public Generic {
        private:
            bool mState;
            bool mPrevState;
            uint32_t mLastChangeMs = 0;
            uint32_t mDebounceTimeMs = 200;
            callback_t mMotionDetected, mMotionCleared;
        public:
            PIR(String name, int16_t id, Buses bus, uint8_t address);
            virtual ~PIR() {}

            inline Classes Class() const override { return Classes::CLASS_PIR; }
            inline bool State() { return mState; }
            inline void DebounceTime(uint32_t ms) { mDebounceTimeMs = ms; }
            
            void Control() override;

            virtual callback_t GetEventCallback(const String& eventName) override;
            virtual bool SetEventCallback(const String& eventName, callback_t cb) override;
    };

    class Button : public Generic {
        private:
            bool mLongClick_Detected_Reported = false;
            bool mPressed_Triggered = false;
            bool mLongClick_Detected = false;
            bool mSequenceSuppressClicks = false;
            ButtonReportModes mReportMode = ButtonReportModes::BUTTONREPORTMODE_CLICKSONLY;
            uint8_t mPressedState = LOW;
            uint8_t mState;
            uint8_t mPrev_State;
            uint8_t mClick_Count = 0;
            ClickTypes mLast_Click_Type = CLICKTYPE_NOCLICK;
            PCF8574* pcf8574;
            uint32_t mDown_Ms;
            uint32_t mDown_Time_Ms = 0;
            uint32_t mClick_Ms;
            uint32_t mLongClick_Detected_Counter;
            callback_t mPressed, mReleased, mClicked, mLongClicked, mDoubleClicked, mTripleClicked;
        public:
            Button(String name, int16_t id, Buses bus, uint8_t address, ButtonReportModes reportmode = ButtonReportModes::BUTTONREPORTMODE_CLICKSONLY);
            virtual ~Button() {}

            inline Classes Class() const override { return CLASS_BUTTON; }
            uint32_t Debounce_Time_Ms = 15;
            uint32_t LongClick_Time_Ms = 300;
            uint32_t DoubleClick_Time_Ms = 300;
            bool LongClick_Detected_Retriggerable = false;
            inline const bool IsPressed() { return (mState == mPressedState); }
            inline const bool IsPressedRaw() { return (digitalRead(mAddress) == mPressedState); }
            void ReportMode(ButtonReportModes mode) { mReportMode = mode; }
            ButtonReportModes ReportMode() { return mReportMode; }
            inline const uint32_t WasPressedFor() { return mDown_Time_Ms; }
            inline const uint8_t NumberOfClicks() { return mClick_Count; }
            inline const ClickTypes ClickType() { return mLast_Click_Type; }
            void Do(ClickTypes click);
            inline bool operator==(Button& rhs) { return (this == &rhs); }
            inline const bool State() { return IsPressed(); }
            void Control() override;

            virtual callback_t GetEventCallback(const String& eventName) override;
            virtual bool SetEventCallback(const String& eventName, callback_t cb) override;
    };

    class Blinds : public Generic {
        private:
            uint8_t mCurrentPosition;
            uint8_t mTargetPosition;
            volatile uint16_t mStepMs = 250;
            BlindsStates mState = BLINDSSTATE_STOPPED;
            volatile bool mCancel = false;
            Relay* mRelayUp;
            Relay* mRelayDown;
            DeviceIQ_DateTime::Timer* mTimerUp;
            DeviceIQ_DateTime::Timer* mTimerDown;
            inline String StateToString(BlindsStates state) { return (state == BLINDSSTATE_DECREASING ? "DECREASING" : (state == BLINDSSTATE_INCREASING ? "INCREASING" : "STOPPED")); }
            callback_t mClosed, mOpened, mBeforeClose, mBeforeOpen;
        public:
            Blinds(String name, int16_t id, Relay* relayup, Relay* relaydown);
            virtual ~Blinds() {}

            inline Relay* RelayUp() const { return mRelayUp; }
            inline Relay* RelayDown() const { return mRelayDown; }

            inline Classes Class() const override { return CLASS_BLINDS; }
            void Position(uint8_t value, bool setformerposition = false);
            inline uint8_t Position() { return mCurrentPosition; }
            inline BlindsStates State() { return mState; }
            inline void Open() { Position(100); }
            inline void Close() { Position(0); }
            inline void StepMs(uint16_t value) { mStepMs = value; mTimerUp->SetTimeout(mStepMs); mTimerDown->SetTimeout(mStepMs); }
            inline uint16_t StepMs() { return mStepMs; }
            inline void CancelAction() { mCancel = true; }
            inline bool operator==(Blinds& rhs) { return (this == &rhs); }
            void Control() override;

            inline bool IsVirtual() const override { return true; }

            virtual callback_t GetEventCallback(const String& eventName) override;
            virtual bool SetEventCallback(const String& eventName, callback_t cb) override;
    };

    class Thermometer : public Generic {
        private:
            enum class DSState { Idle, Waiting };
            DSState mDSState = DSState::Idle;

            uint32_t mReqStartMs = 0;
            uint16_t mConvMaxMs = 188;
            uint32_t mNextDueMs = 0;
            uint32_t mMinPeriodMs = 250;
            uint32_t mMaxPeriodMs = 5000;
            uint32_t mCurrentPeriodMs = 1000;

            ThermometerTypes mType = THERMOMETERTYPE_DHT11;
            OneWire* onewire;
            DallasTemperature* dallastemperature;
            DHT_Unified* dht;
            sensors_event_t dht_event;
            DeviceIQ_DateTime::Timer* Updater_Timer;
            float newTemperature = 0;
            float newHumidity = 0;
            float mTemperature = 0;
            float mHumidity = 0;
            float mTemperatureThreshold = 0.5f;
            callback_t mTemperatureChanged, mHumidityChanged;
        public:
            Thermometer(String name, int16_t id, Buses bus, uint8_t address, ThermometerTypes type);
            virtual ~Thermometer() {}

            inline Classes Class() const override { return CLASS_THERMOMETER; }
            inline float Temperature() { return mTemperature; }
            inline float Humidity() { return mHumidity; }
            TemperatureScales Scale = TEMPERATURESCALE_CELSIUS;
            inline bool operator==(Thermometer& rhs) { return (this == &rhs); }
            inline void Control() override { if (!mEnabled) return; Updater_Timer->Control(); }
            inline void TemperatureThreshold(float v) { mTemperatureThreshold = v; }
            inline float TemperatureThreshold() const { return mTemperatureThreshold; }

            virtual callback_t GetEventCallback(const String& eventName) override;
            virtual bool SetEventCallback(const String& eventName, callback_t cb) override;
    };

    class Currentmeter : public Generic {
        private:
            uint16_t mWindowMs = 100;
            uint16_t mMinSamples = 400;
            float mAlpha = 0.2f;
            float mMvPerAmp = 66.0f;
            int mZeroOffsetmV = 1650;
            bool mAutoCalibrated = false;
            float mThresholdAC = 0.05f;
            float mThresholdDC = 0.05f;
            float newCurrentAC = 0.0f;
            float newCurrentDC = 0.0f;
            float mCurrentAC = 0.0f;
            float mCurrentDC = 0.0f;
            DeviceIQ_DateTime::Timer* Updater;
            callback_t mCurrentACChanged, mCurrentDCChanged;
        public:
            Currentmeter(String name, int16_t id, Buses bus, uint8_t address);
            virtual ~Currentmeter() {}

            inline Classes Class() const override { return CLASS_CURRENTMETER; }
            inline float CurrentAC() { return mCurrentAC; }
            inline float CurrentDC() { return mCurrentDC; }
            inline bool operator==(Currentmeter& rhs) { return (this == &rhs); }
            inline void Control() override { if (!mEnabled) return; Updater->Control(); }
            inline void SetmVperAmp(float mv_per_amp) { mMvPerAmp = mv_per_amp; }
            inline void SetZeroOffsetmV(int offset_mv) { mZeroOffsetmV = offset_mv; mAutoCalibrated = true; }
            void CalibrateZero(uint16_t samples = 2000);
            inline void WindowMs(uint16_t v) { mWindowMs = v; }
            inline void Smoothing(float alpha01) { mAlpha = alpha01; }
            inline void ThresholdAC(float v) { mThresholdAC = v; }
            inline void ThresholdDC(float v) { mThresholdDC = v; }
            inline float ThresholdAC() const { return mThresholdAC; }
            inline float ThresholdDC() const { return mThresholdDC; }

            virtual callback_t GetEventCallback(const String& eventName) override;
            virtual bool SetEventCallback(const String& eventName, callback_t cb) override;
    };

    class Doorbell : public Button {
        private:
            uint8_t mBrightness = 100;
            uint8_t mVolume = 100;
            uint8_t mState = 0; // 0 idle, 1 ring, 2 double, 3 long
            uint32_t mLastEventMs = 0;
            uint32_t mTimeoutMs = 1000; // default 1s
            callback_t mRing, mDoubleRing, mLongRing, mBrightnessChanged, mVolumeChanged;
        public:
            Doorbell(String name, int16_t id, Buses bus, uint8_t address);
            virtual ~Doorbell() {}
            inline Classes Class() const override { return CLASS_DOORBELL; }
            inline uint8_t Brightness() { return mBrightness; }
            inline void Brightness(uint8_t v) { v = constrain(v, 0, 100); if (v != mBrightness) { mBrightness = v; if (mBrightnessChanged) mBrightnessChanged(); } }
            inline uint8_t Volume() { return mVolume; }
            inline void Volume(uint8_t v) { v = constrain(v, 0, 100); if (v != mVolume) { mVolume = v; if (mVolumeChanged) mVolumeChanged(); } }
            inline uint8_t State() { if (mState != 0 && (millis() - mLastEventMs) >= mTimeoutMs) mState = 0; return mState; }
            inline void Timeout(uint32_t ms) { mTimeoutMs = ms; }
            inline uint32_t Timeout() { return mTimeoutMs; }

            virtual callback_t GetEventCallback(const String& eventName) override;
            virtual bool SetEventCallback(const String& eventName, callback_t cb) override;
    };

    class ContactSensor : public Button {
        private:
            bool mInvertClosed = false;
            callback_t mOpened, mClosed;
        public:
            ContactSensor(String name, int16_t id, Buses bus, uint8_t address, bool invertClosed = false);
            virtual ~ContactSensor() {}
            inline Classes Class() const override { return CLASS_CONTACTSENSOR; }
            inline bool State() { bool pressed = IsPressed(); return mInvertClosed ? !pressed : pressed; }
            inline bool IsClosed() { return State(); }
            inline bool IsOpen()   { return !State(); }
            inline void InvertClosed(bool v) { mInvertClosed = v; }

            virtual callback_t GetEventCallback(const String& eventName) override;
            virtual bool SetEventCallback(const String& eventName, callback_t cb) override;
    };

    class Collection {
        private:
            RunState mState = RunState::Running;
            bool mMutating = false;
            std::vector<Generic*> mCollection;

            inline void Pause() { mState = RunState::Paused; }
            inline void Resume() { mState = RunState::Running; }

        public:
            bool Add(Generic* new_component);
            inline bool IsPaused() const { return mState == RunState::Paused; }
            inline bool IsMutating() const { return mMutating; }

            inline int16_t IndexOf(const String& name) { for (int16_t i = 0; i < (int16_t)mCollection.size(); ++i) { if (mCollection[i] && mCollection[i]->Name().equalsIgnoreCase(name)) return i; } return -1; }

            int16_t Remove(int16_t index);
            inline int16_t Remove(const String& name) { int16_t idx = IndexOf(name); return (idx >= 0) ? Remove(idx) : -1; }

            void Clear();

            inline Generic* At(int16_t index) { if (index < 0 || index >= (int16_t)mCollection.size()) return nullptr; return mCollection[index]; }
            inline size_t Count() const { return mCollection.size(); }

            inline Generic* operator[](int16_t index) { return At(index); }
            inline Generic* operator[](const String& name) { return At(IndexOf(name)); }

            inline std::vector<Generic*>::iterator begin() { return mCollection.begin(); }
            inline std::vector<Generic*>::iterator end() { return mCollection.end(); }
            inline std::vector<Generic*>::const_iterator begin() const { return mCollection.begin(); }
            inline std::vector<Generic*>::const_iterator end() const { return mCollection.end(); }

            inline void Refresh() { if (mMutating) return; for (auto* m : mCollection) { if (m) m->Refresh(); }}
            
            void Control();
    };
}

#endif