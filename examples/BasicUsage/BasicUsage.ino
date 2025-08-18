#include <Arduino.h>
#include <DevIQ_Components.h>

using namespace DeviceIQ_Components;

Collection devCollection;

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Create a relay component
    auto relay = new Relay("Relay1", 1, BUS_ONBOARD, 5, RELAYTYPE_NORMALLYOPENED);
    relay->Event["Changed"]([] { Serial.println("Relay state changed!"); });
    devCollection.Add(relay);

    // Create a thermometer component
    auto temp = new Thermometer("Temp1", 2, BUS_ONBOARD, 4, THERMOMETERTYPE_DHT22);
    temp->TemperatureThreshold(0.5f);
    temp->OnTemperatureChanged([] {
        Serial.println("Temperature changed!");
    });
    devCollection.Add(temp);

    // Create a current meter component
    auto current = new Currentmeter("Current1", 3, BUS_ONBOARD, 34); // ADC pin
    current->ThresholdAC(0.05f);
    current->ThresholdDC(0.05f);
    current->OnCurrentACChanged([] {
        Serial.println("AC current changed!");
    });
    current->OnCurrentDCChanged([] {
        Serial.println("DC current changed!");
    });
    devCollection.Add(current);
}

void loop() {
    // Periodically update all components
    devCollection.Control();
}