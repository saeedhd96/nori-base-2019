
#ifndef homo
#define homo


#include <nori/proplist.h>
#include <math.h>
#include <nori/common.h>
#include <nori/sampler.h>
//#include "MediumIntersection.h"
#include <nori/object.h>
#include "nori/medium.h"


using namespace std;
NORI_NAMESPACE_BEGIN

Medium::Medium(const PropertyList &propList) {
    sigma_s = propList.getColor("sigmaS", Color3f(1.f));
    sigma_a = propList.getColor("sigmaA", Color3f(0.01));
    Lve = propList.getColor("Lve", Color3f(0.f));
    g = propList.getFloat("g", 0);
}




Color3f Medium::p(const Vector3f &wi, const Vector3f &wo ) const {
            float cosTheta = wi.dot(wo);
            float denom = 1 + g * g + 2 * g * cosTheta;
            return INV_PI/4 * (1 - g * g) / (denom * std::sqrt(denom));
}

    bool Medium::isHomo1() const {
        return isHomo;
    }

    void Medium::setIsHomo(bool isHomo) {
        Medium::isHomo = isHomo;
    }


//        float pdf() const {
//            return 0.0f;
//        }

//        Color3f sample(Ray3f ray_, Sampler sampler) const {
//            int channel = ((int)sampler.next1D()*3)+1;
//            float l = ray_.d[0]*ray_.d[0]+ray_.d[1]*ray_.d[1] + ray_.d[2]*ray_.d[2];
//            l  = sqrt(l);
//            float t = min(-log(1-sampler.next1D())*l/sigma_t[channel], ray_.maxt);
//            bool isInMeduim = t<ray_.maxt;
//            MediumIntersection *a= nullptr;
//            if(isInMeduim) {
//                a = new MediumIntersection();
//                a->medium= this;
//                a->p = ray_(t);
//                a->wo = -ray_.d;
//            }

//            Color3f L = Exp(-sigma_t * min(t, MaxFloat) * ray.d.Length())
//            return Color3f(0.f);

//        }




NORI_REGISTER_CLASS(Medium, "homo");
NORI_NAMESPACE_END

#endif