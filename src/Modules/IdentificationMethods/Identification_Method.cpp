#include "Identification_Method.h"

MethodState Identification_Method::GetState() {
    return state;
}

LogLevels Identification_Method::GetLogLevel() {
    return logLevel;
}

void Identification_Method::SetLogLevel(LogLevels newLogLevel) {
    logLevel = newLogLevel;
}

double Identification_Method::GetTrustValue() {
    return trustValue;
}

void Identification_Method::SetTrustValue(double newTrustValue) {
    trustValue = newTrustValue;
}