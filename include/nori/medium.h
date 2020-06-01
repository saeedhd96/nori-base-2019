//
// Created by Saeed HD on 5/23/20.
//

#ifndef NORI_MEDIUM_H
#define NORI_MEDIUM_H

#include "nori/object.h"

NORI_NAMESPACE_BEGIN
class Medium : public NoriObject {

public:
    Color3f sigma_a, sigma_s,Lve;
    float g;

    Medium(const PropertyList &propList);
    Color3f p(const Vector3f &wi, const Vector3f &wo ) const ;

    EClassType getClassType() const { return EMedium; }

    std::string toString() const {
        return tfm::format(
                "Medium[\n"
                "  sigma_a = %f,\n"
                "  sigma_s = %f\n"
                "]",
                sigma_a, sigma_s);
    }


    bool isHomo1() const;

    void setIsHomo(bool isHomo);

private:
    bool isHomo = true;

};

NORI_NAMESPACE_END
#endif //NORI_MEDIUM_H
