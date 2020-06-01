//
// Created by Saeed HD on 3/9/20.
//


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

#pragma once

#include <nori/emitter.h>


NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter {
    public:
        uint32_t getTriangleCount() const {  }
    AreaLight(const PropertyList &props) {
             radiance = props.getColor("radiance");
    }

    std::string toString() const {
        return "AreaLight[]";
    }
    Color3f eval(Point3f &p, Vector3f &normals, Vector3f sampledToIntersection) const {
            if (sampledToIntersection.dot(normals)<=0) return Color3f(0.f);

            return radiance;
    }

    const Color3f &getRadiance() const {
        return radiance;
    }

private:
        Color3f radiance;
    };




NORI_REGISTER_CLASS(AreaLight,"area")
NORI_NAMESPACE_END
