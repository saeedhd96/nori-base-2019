//
// Created by Saeed HD on 3/9/20.
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

class WhittedIntegrator : public Integrator {
    public:
        WhittedIntegrator(const PropertyList &props) {

        }

//    float calcGeoTerm( Normal3f nx, Normal3f ny, Vector3f yx,float sizeYX2) const {
//
//        return abs(nx.dot(-yx))*abs(ny.dot(yx))/sizeYX2;

//    }

    Color3f Li(const Scene *scene, Sampler *samplerr, const Ray3f &ray) const {
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);
            /* Return the component-wise absolute
               value of the shading normal as a color */

            float emitterCount;
            Mesh *areaLight = scene->getSampleEmitter(emitterCount);
            Point3f lightPointSampled;
            Vector3f lightPointSampledNormals;
            float lightPointSampledPDF;
            areaLight->sampler(samplerr->next1D(),samplerr->next1D(),lightPointSampled,lightPointSampledNormals,lightPointSampledPDF);

            Vector3f yx = (its.p-lightPointSampled);
            Vector3f nyx = (its.p-lightPointSampled).normalized();
            Normal3f nx = its.shFrame.n;
            Normal3f ny = lightPointSampledNormals;
            Color3f Le = areaLight->getEmitter()->eval(lightPointSampled, lightPointSampledNormals,nyx);
            Color3f L = Color3f{0.0f};
            if (its.mesh->isEmitter()) L = its.mesh->getEmitter()->getRadiance();

            Ray3f rayXY(its.p, -nyx);
            Intersection itsOnSource;
            float geoTerm = 0;
            if (!scene->rayIntersect(rayXY, itsOnSource) || itsOnSource.mesh != areaLight){
                return L;
            }

            Vector3f wiii = (lightPointSampled - its.p).normalized();
            geoTerm = std::abs(its.shFrame.n.dot(wiii) * lightPointSampledNormals.dot(-wiii) / (lightPointSampled-its.p).dot(lightPointSampled-its.p));;


            const BSDF* bsdf = its.mesh->getBSDF();
            Normal3f wo = its.shFrame.toLocal(-ray.d);
            Normal3f wi = its.shFrame.toLocal(-nyx);


            Color3f bsdfResult = bsdf->eval(BSDFQueryRecord(wi,wo,ESolidAngle));

        Color3f Lr = geoTerm*Le;
            Lr /= lightPointSampledPDF;
            Lr *= emitterCount;

            Color3f Li( Lr.x()*bsdfResult.x(),Lr.y()*bsdfResult.y(),Lr.z()*bsdfResult.z());
        return L+Li;
        }


    std::string toString() const {
            return "WhittedIntegrator[]";
        }


    };
NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END
