#include "Identification_Method.h"

MethodState Identification_Method::GetState() {
    return state;
}

double Identification_Method::GetTrustValue() {
    return trustValue;
}

void Identification_Method::SetTrustValue(double newTrustValue) {
    trustValue = newTrustValue;
}