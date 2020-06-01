/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    float a = ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
    return a;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
    throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float a1 = sample[0];
    float a2 = sample[1];
    float sq= sqrt(a2);
    float x = cos(2*M_PI*a1)*sq;
    float y = sin(2*M_PI*a1)*sq;

    return Point2f(x,y);
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
     float b = (p[0]*p[0]+p[1]*p[1]<1.0) ? 1.0/M_PI : 0.0f;
     return  b;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
}
float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float a1 = sample[0];
    float a2 = sample[1];
    float sq= sqrt(a2);
    float x = cos(2*M_PI*a1)*sq;
    float y = sin(2*M_PI*a1)*sq;
    float z = sqrt(1-a2);
    return Vector3f(x,y,z);
}


float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    float b = v[2]>=0?v[2]/M_PI:0.0;
    return  b;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {

    float a1 = sample[0];
    float a2 = sample[1];
    float phi = 2*M_PI*a1;
    float theta = abs(atan(sqrt(-alpha*alpha*log(1-a2))));
    return Vector3f(cos(phi)*sin(theta), sin(phi)*sin(theta), cos(theta) );
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
//    float cos_phi__sin_theta = m[0];
//    float sin_phi__sin_theta= m[1];
    float cos_theta = m[2];
    float tan2_theta = (1-m[2]*m[2])/(m[2]*m[2]);
    float alpha2 = alpha*alpha;
    if (cos_theta<=0.0)return 0.0;
    float longPart = 2*exp(-tan2_theta/alpha2)/(alpha2*cos_theta*cos_theta*cos_theta);
    float azimPart = 1/(2*M_PI);

    return longPart*azimPart;

}

NORI_NAMESPACE_END
