# DeviceIQ Lib Components

`DeviceIQ Lib Components` is a component management library for automation systems based on ESP32/ESP8266.  
It provides classes for controlling actuators and reading sensors with event support, configurable thresholds, and seamless integration into automation engines.

It includes implementations for relays, buttons, blinds, thermometers, and AC/DC current meters, with direct reading via GPIO, I2C, and other buses, allowing you to attach callbacks to events such as state changes, readings, and automatic actions.

The library is modular and easily extensible, enabling you to add new component types as needed.

Features
--------
- Support for multiple component types: Relay, Button, Blinds, Thermometer, Currentmeter.
- Sensor readings with configurable thresholds for event triggering.
- Actuator control via GPIO or I2C buses.
- Internal timing support (`DeviceIQ_DateTime::Timer`) for polling and periodic events.
- Extensible architecture for adding new components.
- Direct integration into event/action-based automation systems.

Usage Example
-------------

```cpp
#include <DevIQ_Components.h>

using namespace DeviceIQ_Components;

Collection devCollection;

void setup() {
    auto relay = new Relay("Relay1", 1, BUS_ONBOARD, 5, RELAYTYPE_NORMALLYOPENED);
    relay->Event["Changed"]([] { Serial.println("Relay state changed!"); });
    devCollection.Add(relay);

    auto temp = new Thermometer("Temp1", 2, BUS_ONBOARD, 4, THERMOMETERTYPE_DHT22);
    temp->TemperatureThreshold(0.5f);
    temp->OnTemperatureChanged([] { Serial.println("Temperature changed!"); });
    devCollection.Add(temp);
}

void loop() {
    devCollection.Control();
}
```

API Summary
-----------

### Main Classes

| Class | Description |
|-------|-------------|
| `Relay` | Relay control with events for state changes. |
| `Button` | Physical button reading with single, double, triple, and long click detection. |
| `Blinds` | Blind/shade control with percentage position and open/close events. |
| `Thermometer` | Temperature/humidity reading with configurable event threshold. |
| `Currentmeter` | AC/DC current measurement with event thresholds. |
| `Collection` | Manages a set of components, allowing batch control. |

### Common Methods

| Method | Description |
|--------|-------------|
| `Control()` | Updates the component state (should be called in the loop). |
| `Refresh()` | Reloads data or component state. |
| `Event["Name"]([]{})` | Binds a callback to a supported component event. |

### Threshold Methods

| Method | Applies To | Description |
|--------|------------|-------------|
| `TemperatureThreshold(float v)` | Thermometer | Defines the minimum delta to trigger a temperature event. |
| `ThresholdAC(float v)` | Currentmeter | Defines the minimum delta to trigger an AC current event. |
| `ThresholdDC(float v)` | Currentmeter | Defines the minimum delta to trigger a DC current event. |

Notes
-----
## Dependencies
- [ArduinoJson](https://arduinojson.org/) (v7)
- [OneWire](https://github.com/PaulStoffregen/OneWire)
- [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)
- [PCF8574](https://github.com/mathertel/PCF8574)
- [DHT sensor library](https://github.com/adafruit/DHT-sensor-library)

License
-------
This library is released under the MIT License.
