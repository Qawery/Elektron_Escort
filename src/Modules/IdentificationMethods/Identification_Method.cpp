#include "Identification_Method.h"

MethodState Identification_Method::GetState() {
    return state;
}

LogLevels Identification_Method::GetLogLevel() {
    return logLevel;
}

void Identification_Method::SetLogLEvel(LogLevels newLogLevel) {
    logLevel = newLogLevel;
}