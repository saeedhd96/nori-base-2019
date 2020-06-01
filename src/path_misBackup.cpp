//
// Created by Saeed HD on 4/7/20.
//

//
// Created by Saeed HD on 4/6/20.
//


//
// Created by Saeed HD on 4/6/20.
//

#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

#include <math.h>
#include <vector>

using namespace std;

NORI_NAMESPACE_BEGIN

    class PathMIS : public Integrator {
    public:
        PathMIS(const PropertyList &props) {

        }



        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);
            /* Return the component-wise absolute
             * value of the shading normal as a color */
            int k = 0;
            float q = 1.f;
            float etas = 1.f;
            float pLight = 0.5;
            Color3f alpha(1.f);
            Color3f L(0.f);
            Normal3f w_i(ray.d.normalized());
            if (its.mesh->isEmitter()) return its.mesh->getEmitter()->getRadiance();
            Ray3f finalRay = ray;
            bool wasRefractive = false;
            bool firstTime = true;
            while (true) {
                float x = sampler->next1D();
                Color3f hitEmmiter(0.f);
                Color3f shadow(0.f);
                if (its.mesh->isEmitter() ){
                    if(wasRefractive) L += its.mesh->getEmitter()->getRadiance()*alpha;
                } else {
                    if (its.mesh->getBSDF()->isDiffuse()) {
                        BSDFQueryRecord record(NULL);
                        Color3f ems(0.f), mats(0.f);
                        Color3f emitter;
                        float emsProb;
                        ems += samplePointandReturnL(scene, sampler, finalRay, its, record, emitter, emsProb ) * alpha;
                        mats += emitter * its.mesh->getBSDF()->eval(record)*abs(record.wi.z()) * alpha;
                        float plight = 1/(emsProb+its.mesh->getBSDF()->pdf(record));
                        L+= mats*(plight)+ ems*(1-pLight);
                    }


                }






                if (x < q) {
                    w_i = its.shFrame.toLocal(-w_i);
                    BSDFQueryRecord query(w_i, NULL, ESolidAngle);
                    Color3f alphaa = its.mesh->getBSDF()->sample(query, sampler->next2D());






                    alpha *= alphaa;
                    alpha /= q;


                    w_i = its.shFrame.toWorld(query.wo);
                    finalRay =  Ray3f(its.p,w_i);
                    wasRefractive = !its.mesh->getBSDF()->isDiffuse();
                    scene->rayIntersect(finalRay,its);




                    k++;


                    float maxComponentThroughput = alpha.maxCoeff();
                    etas *= query.eta;
                    float xx= maxComponentThroughput*etas*etas;

                    if (!firstTime) {
                        q = 0.99 < xx ? 0.99 : xx;
                    }else firstTime= false;

                }else {
                    return L;
                }

            }
        }

        Color3f samplePointandReturnL(const Scene *scene, Sampler *samplerr, Ray3f &ray, const Intersection &its,
                                      BSDFQueryRecord &rec,
                                      Color3f &f, float &emsProb) const {
            float emitterCount;
            Mesh *areaLight = scene->getSampleEmitter(emitterCount);
            Point3f lightPointSampled;
            Vector3f lightPointSampledNormals;
            float lightPointSampledPDF;
            areaLight->sampler(samplerr->next1D(), samplerr->next1D(), lightPointSampled, lightPointSampledNormals,
                               lightPointSampledPDF);

            Vector3f yx = (its.p - lightPointSampled);
            Vector3f nyx = (its.p - lightPointSampled).normalized();
            Normal3f nx = its.shFrame.n;
            Normal3f ny = lightPointSampledNormals;
            Color3f Le = areaLight->getEmitter()->eval(lightPointSampled, lightPointSampledNormals, nyx);
            f=Le;
//            Color3f L = Color3f{0.0f};
//            if (its.mesh->isEmitter()) L = its.mesh->getEmitter()->getRadiance();

            Ray3f rayXY(its.p, -nyx);
            Intersection itsOnSource;
            float geoTerm = 0;
            if (!scene->rayIntersect(rayXY, itsOnSource) || itsOnSource.mesh != areaLight) {
                return Color3f(0.f);
            }

            Vector3f wiii = (lightPointSampled - its.p).normalized();
            geoTerm = std::abs(its.shFrame.n.dot(wiii) * lightPointSampledNormals.dot(-wiii) /
                               (lightPointSampled - its.p).dot(lightPointSampled - its.p));;


            const BSDF *bsdf = its.mesh->getBSDF();
            Normal3f wo = its.shFrame.toLocal(-ray.d);
            Normal3f wi = its.shFrame.toLocal(-nyx);

             BSDFQueryRecord que(wi, wo, ESolidAngle);
             rec = que ;

             emsProb = lightPointSampledPDF/(abs(lightPointSampledNormals.dot(nyx))*(sqrt(yx.y()*yx.y()+yx.z()*yx.z()+yx.x()*yx.x())));

            Color3f bsdfResult = bsdf->eval(que);

            Color3f Lr = geoTerm * Le;
            Lr /= lightPointSampledPDF;
            Lr *= emitterCount;

            Color3f Li(Lr.x() * bsdfResult.x(), Lr.y() * bsdfResult.y(), Lr.z() * bsdfResult.z());
            return Li;
        }


        std::string toString() const {
            return "PathEventEstimation[]";
        }


    };
    NORI_REGISTER_CLASS(PathMIS, "path_mis");
NORI_NAMESPACE_END


