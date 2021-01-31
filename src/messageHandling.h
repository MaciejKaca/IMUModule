#include <Arduino.h>

bool isMessagePresent()
{
    if(Serial.available() )
    {
        return true;
    }
    return false;
}

template<class Message>
void receiveInitialMessage(Message &message)
{
    if(typeid (T) == typeid(SignalCfm))
    Serial.readBytes((char *) &message, sizeof(Message));
}