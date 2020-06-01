/*

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

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

    class Microfacet : public BSDF {
    public:
        Microfacet(const PropertyList &propList) {
            /* RMS surface roughness */
            m_alpha = propList.getFloat("alpha", 0.1f);

            /* Interior IOR (default: BK7 borosilicate optical glass) */
            m_intIOR = propList.getFloat("intIOR", 1.5046f);

            /* Exterior IOR (default: air) */
            m_extIOR = propList.getFloat("extIOR", 1.000277f);

            /* Albedo of the diffuse base material (a.k.a "kd") */
            m_kd = propList.getColor("kd", Color3f(0.5f));

            /* To ensure energy conservation, we must scale the
               specular component by 1-kd.
               While that is not a particularly realistic model of what
               happens in reality, this will greatly simplify the
               implementation. Please see the course staff if you're
               interested in implementing a more realistic version
               of this BRDF. */
            m_ks = 1 - m_kd.maxCoeff();
        }



        float G1(const Vector3f wv, const Vector3f wh) const{
            float tan_theta_v = Frame::sinTheta(wv) / Frame::cosTheta(wv);

            float b = 1/(m_alpha * tan_theta_v);
            float func_1, func_2;

            if (wv.dot(wh) / Frame::cosTheta(wv) > 0){
                func_1 = 1;
            } else {
                func_1 = 0;
            }

            if (b < 1.6){
                func_2 = (3.535f * b + 2.181f * b * b) / (1 + 2.276f * b + 2.577f * b * b);
            } else{
                func_2 = 1.0f;
            }

            return func_1 * func_2;

        }

        float G(const Vector3f wi, const Vector3f wo, const Vector3f wh) const{
            return G1(wi, wh) * G1(wo, wh);
        }


        /// Evaluate the BRDF for the given pair of directions
        Color3f eval(const BSDFQueryRecord &bRec) const {

            if ( Frame::cosTheta(bRec.wi) <= 0.0f
                 || Frame::cosTheta(bRec.wo) <= 0.0f){
                return Color3f{0.0f};
            }

            Vector3f wh = (bRec.wi + bRec.wo);
            wh = wh.normalized();

            float D = Warp::squareToBeckmannPdf(wh, m_alpha);
            float F = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);
            float numerator = m_ks * D * F * G(bRec.wi, bRec.wo, wh);
            float denominator = 4 * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo) * Frame::cosTheta(wh);

            Color3f fr = m_kd / M_PI + numerator/denominator;

            return fr;

        }

        /// Evaluate the sampling density of \ref sample() wrt. solid angles
        float pdf(const BSDFQueryRecord &bRec) const {

            if (Frame::cosTheta(bRec.wi) <= 0
                || Frame::cosTheta(bRec.wo) <= 0){
                return 0;
            }

            Vector3f wh = (bRec.wi + bRec.wo);
            wh = wh.normalized();
            float D = Warp::squareToBeckmannPdf(wh, m_alpha);
            float J_h = 1 / (4 * wh.dot(bRec.wo));
            float pdf = m_ks * D * J_h + (1 - m_ks) * Frame::cosTheta(bRec.wo) / M_PI;
            return pdf;
        }

        /// Sample the BRDF
        Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {


            if(Frame::cosTheta(bRec.wi) <= 0.0f) {
                return Color3f(0.0f);
            }

            bRec.measure = ESolidAngle;
            Point2f sample = _sample;

            if (sample.x() < m_ks) {

                // scaling
                sample.x() = sample.x() / m_ks;

                Vector3f wn = Warp::squareToBeckmann(sample, m_alpha);
                bRec.wo = 2 * wn * bRec.wi.dot(wn) - bRec.wi;
            }
            else {
                sample.x() = (sample.x() - m_ks) / (1 - m_ks);
                bRec.wo = Warp::squareToCosineHemisphere(sample);
            }

            if (pdf(bRec) > 0) return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
            else {
                return Color3f{0.0f};
            }

            // Note: Once you have implemented the part that computes the scattered
            // direction, the last part of this function should simply return the
            // BRDF value divided by the solid angle density and multiplied by the
            // cosine factor from the reflection equation, i.e.
            // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        }

        bool isDiffuse() const {
            /* While microfacet BRDFs are not perfectly diffuse, they can be
               handled by sampling techniques for diffuse/non-specular materials,
               hence we return true here */
            return true;
        }

        std::string toString() const {
            return tfm::format(
                    "Microfacet[\n"
                    "  alpha = %f,\n"
                    "  intIOR = %f,\n"
                    "  extIOR = %f,\n"
                    "  kd = %s,\n"
                    "  ks = %f\n"
                    "]",
                    m_alpha,
                    m_intIOR,
                    m_extIOR,
                    m_kd.toString(),
                    m_ks
            );
        }
    private:
        float m_alpha;
        float m_intIOR, m_extIOR;
        float m_ks;
        Color3f m_kd;
    };

    NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END







//    This file is part of Nori, a simple educational ray tracer
//
//    Copyright (c) 2015 by Wenzel Jakob
//
//    Nori is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License Version 3
//    as published by the Free Software Foundation.
//
//    Nori is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program. If not, see <http://www.gnu.org/licenses/>.
//*/
//
//#include <nori/bsdf.h>
//#include <nori/frame.h>
//#include <nori/warp.h>
//
//NORI_NAMESPACE_BEGIN
//
//class Microfacet : public BSDF {
//public:
//    Microfacet(const PropertyList &propList) {
//        /* RMS surface roughness */
//        m_alpha = propList.getFloat("alpha", 0.1f);
//
//        /* Interior IOR (default: BK7 borosilicate optical glass) */
//        m_intIOR = propList.getFloat("intIOR", 1.5046f);
//
//        /* Exterior IOR (default: air) */
//        m_extIOR = propList.getFloat("extIOR", 1.000277f);
//
//        /* Albedo of the diffuse base material (a.k.a "kd") */
//        m_kd = propList.getColor("kd", Color3f(0.5f));
//
//        /* To ensure energy conservation, we must scale the
//           specular component by 1-kd.
//
//           While that is not a particularly realistic model of what
//           happens in reality, this will greatly simplify the
//           implementation. Please see the course staff if you're
//           interested in implementing a more realistic version
//           of this BRDF. */
//        m_ks = 1 - m_kd.maxCoeff();
//    }
//
//
//    float GFunction(Vector3f wv,Vector3f wh) const {
//        Vector3f n(1,0,0);
//        if (wv.dot(wh)/(wv.dot(n))<0) return 0.f;
//        else {
//            float b = 1 / (m_alpha * tan(2));
//            if (b > 1.6)
//                return (3.535 * b + 2.18 * b * b) / (1 + 2.276 * b + 2.577 * b * b);
//            else return 1.f;
//        }
//    }
//
//
//        /// Evaluate the BRDF for the given pair of directions
//    Color3f eval(const BSDFQueryRecord &bRec) const {
//        Color3f f = m_kd/M_PI;
//        Warp *warp = new Warp();
//
//        Vector3f wh = (bRec.wi+bRec.wo)/((bRec.wi+bRec.wo).dot(bRec.wi+bRec.wo));
//        float Frenscel = fresnel(wh.dot(bRec.wi),m_extIOR,m_intIOR);
//        Color3f beckmann = warp->squareToBeckmannPdf(wh,1.f);
//
//
//        Frame whframe(wh),woframe(bRec.wo),wiframe(bRec.wi);
////        Vector3f localWh(whframe.toLocal(wh)),localWo(woframe.toLocal()),localWi(bRec.wi);
//        float denom = 4*whframe.toLocal(wh).z()*whframe.toLocal(bRec.wi).z()*whframe.toLocal(bRec.wo).z();
//        float g1 = GFunction(bRec.wo,wh);
//        float g2 = GFunction(bRec.wi,wh);
//        f+=m_ks*beckmann*Frenscel*g1*g2/denom;
//            return f;
//    }
//
//    /// Evaluate the sampling density of \ref sample() wrt. solid angles
//    float pdf(const BSDFQueryRecord &bRec) const {
//    	throw NoriException("MicrofacetBRDF::pdf(): not implemented!");
//    }
//
//    /// Sample the BRDF
//    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
//    	throw NoriException("MicrofacetBRDF::sample(): not implemented!");
//
//        // Note: Once you have implemented the part that computes the scattered
//        // direction, the last part of this function should simply return the
//        // BRDF value divided by the solid angle density and multiplied by the
//        // cosine factor from the reflection equation, i.e.
//        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
//    }
//
//    bool isDiffuse() const {
//        /* While microfacet BRDFs are not perfectly diffuse, they can be
//           handled by sampling techniques for diffuse/non-specular materials,
//           hence we return true here */
//        return true;
//    }
//
//    std::string toString() const {
//        return tfm::format(
//            "Microfacet[\n"
//            "  alpha = %f,\n"
//            "  intIOR = %f,\n"
//            "  extIOR = %f,\n"
//            "  kd = %s,\n"
//            "  ks = %f\n"
//            "]",
//            m_alpha,
//            m_intIOR,
//            m_extIOR,
//            m_kd.toString(),
//            m_ks
//        );
//    }
//private:
//    float m_alpha;
//    float m_intIOR, m_extIOR;
//    float m_ks;
//    Color3f m_kd;
//};
//
//NORI_REGISTER_CLASS(Microfacet, "microfacet");
//NORI_NAMESPACE_END
