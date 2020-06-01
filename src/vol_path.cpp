//
// Created by Saeed HD on 5/22/20.
//

#include "vol_path.h"

#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

#include <math.h>
#include <vector>

using namespace std;

const int n =10;
int correction = 1;
float correctL = 1;
float handleSingleScattering = true;
float hetero = false;

NORI_NAMESPACE_BEGIN

    class VolPath : public Integrator {
    public:
        VolPath(const PropertyList &props) {

        }

        bool sampleBSDFOut(Sampler *sampler, const Intersection &its, Normal3f &w_i, Ray3f &newRay, Color3f &alpha, float &eta, bool &specularBounce) const {
            BSDFQueryRecord query(w_i, NULL, ESolidAngle);
            Color3f f= its.mesh->getBSDF()->sample(query, sampler->next2D());
            if (f.isZero())
                return true;
            w_i = its.shFrame.toWorld(query.wo);
            eta = query.eta;
            specularBounce = !its.mesh->getBSDF()->isDiffuse();
            newRay = Ray3f(its.p, w_i);
            alpha*=f;
            return false;
        }

        float density(const Vector3f& p,bool isHomo,const Scene *scene) const{
            if (isHomo)
                return 1.f;
            Vector3f range = scene->getBoundingBox().max-scene->getBoundingBox().min;
            Vector3f dis = (p - scene->getBoundingBox().min);
            float d = sqrt(dis.dot(dis)/range.dot(range));
            float f = abs(3*sin(15*d*M_PI));
            return f;
        }



        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray)  const{
            Intersection its;

            Medium *medium = scene->getSMedium();
            Color3f Lve = medium->Lve;
            medium->setIsHomo(!hetero);
            Color3f sigma_t = medium->sigma_a+medium->sigma_s;

            bool specularBounce= false, foundIntersection= false;
            int k = 0;
            float q = 1.f, etas = 1.f;
            Color3f alpha(1.f), L(0.f);
            Ray3f newRay = ray;
            Normal3f w_i(newRay.d.normalized());
            for (k = 0; ; k++) {
                its = Intersection();
                foundIntersection = scene->rayIntersect(newRay, its);
                if (!foundIntersection || k>=500) break;
                w_i = its.shFrame.toLocal(-w_i);


                if (handleSingleScattering) {
                    Color3f L_s = sampleLighterAndReturnL(scene, sampler, newRay, its, handleSingleScattering,medium);

                    float dt = (its.t - newRay.mint)/n;
                    dt/=correction;
                    Color3f L_raymarch = rayMarch(its,sigma_t,n,newRay,scene,Lve,alpha,medium->isHomo1());
                    L_raymarch*=dt;
                    L += L_raymarch+alpha* L_s;

                }

                if ( specularBounce || k ==0){
                    if (foundIntersection && its.mesh->isEmitter()) {
                        Color3f emmiterL = its.mesh->getEmitter()->getRadiance();
                        L += emmiterL * alpha;
                    }
                }
                L += sampleLighterAndReturnL(scene, sampler, newRay, its, false, nullptr) * alpha;


                //Sample outgoing ray and find its intersection with the scene
                float eta;
                if(sampleBSDFOut(sampler, its, w_i, newRay, alpha,eta, specularBounce)) break;

                if(russianRoulette(k, etas,eta, q, alpha,sampler)) break;
            }

                return L*correctL;
        }

        Color3f rayMarch(const Intersection &its, const Color3f &sigma_t, int n,
                         const Ray3f &newRay, const Scene *scene, const Color3f &L_ve,Color3f &Tr, bool isHomo) const {
            Color3f L(0.f);
            float dt = (its.t - newRay.mint)/n;
            dt/=correction;
            for (int i = 0; i<n ; i++){
                L+=Tr*L_ve;
                Vector3f d = newRay(dt*correction*i);

                Color3f currSigmaT = sigma_t*(density(d,isHomo,scene));
                Color3f currentT=1-currSigmaT*dt;
                Tr*=currentT;

            }
            return L;
        }

        bool russianRoulette(int &k, float &etas, float eta, float &q,
                             Color3f &alpha,  Sampler *sampler) const {
            if (k>3) {
                float maxComponentThroughput = alpha.maxCoeff();
                etas *= eta;
                float xx = maxComponentThroughput * etas * etas;
                q = 0.95 < xx ? 0.95 : xx;
                if (sampler->next1D()>q)
                    return true;
                alpha *= 1 / q;
            }
            return false;
        }

        std::string toString() const {
            return "PathMatsIntegrator[]";
        }
//        Color3f samplePointandReturnL(const Scene *scene, Sampler *samplerr, const Ray3f &ray, const Intersection &its, Color3f sigma_t) const {
//            float emitterCount;
//            Mesh *areaLight = scene->getSampleEmitter(emitterCount);
//            Point3f lightPointSampled;
//            Vector3f lightPointSampledNormals;
//            float lightPointSampledPDF;
//            areaLight->sampler(samplerr->next1D(), samplerr->next1D(), lightPointSampled, lightPointSampledNormals,
//                               lightPointSampledPDF);
//
//            Vector3f nyx = (its.p - lightPointSampled).normalized();
//            Normal3f nx = its.shFrame.n;
//            Normal3f ny = lightPointSampledNormals;
//            Color3f Le = areaLight->getEmitter()->eval(lightPointSampled, lightPointSampledNormals, nyx);
//
//            Ray3f rayXY(its.p, -nyx);
//            Ray3f rayYX(lightPointSampled, nyx);
//            Intersection itsOnSource;
//            Intersection itsOnObject;
//            if ((!scene->rayIntersect(rayXY, itsOnSource) || !itsOnSource.p.isApprox(lightPointSampled,0.00001)) || (!scene->rayIntersect(rayYX, itsOnObject)  || !itsOnObject.p.isApprox( its.p,0.00001))) {
//                return Color3f(0.f);
//            }
//
//            Normal3f wo = its.shFrame.toLocal(-ray.d);
//            Normal3f wi = its.shFrame.toLocal(-nyx);
//            Color3f Tr(1.f);
//            rayMarch(itsOnSource, sigma_t, n, rayXY, scene,Color3f(0.f),Tr);
//            Color3f phase = scene->getSMedium()->p(wi, wo);
//            Color3f Lr = Le;
//            Lr /= (lightPointSampledPDF+phase);
//
//            phase*= scene->getSMedium()->sigma_s;
//            Lr *= emitterCount;
//            Lr*=Tr;
//
//            Color3f Li(Lr.x() * phase.x(),  Lr.y() * phase.y(), Lr.z() * phase.z());
//            return Li;
//        }


Color3f sampleLighterAndReturnL(const Scene *scene, Sampler *samplerr, const Ray3f &ray, const Intersection &its, bool inMedium, const Medium *medium) const {
            float emitterCount;
            Mesh *areaLight = scene->getSampleEmitter(emitterCount);
            Point3f lightPointSampled;
            Vector3f lightPointSampledNormals;
            float lightPointSampledPDF;
            areaLight->sampler(samplerr->next1D(), samplerr->next1D(), lightPointSampled, lightPointSampledNormals,
                               lightPointSampledPDF);

            Vector3f nyx = (its.p - lightPointSampled).normalized();
            Normal3f nx = its.shFrame.n;
            Normal3f ny = lightPointSampledNormals;
            Color3f Le = areaLight->getEmitter()->eval(lightPointSampled, lightPointSampledNormals, nyx);

            Ray3f rayXY(its.p, -nyx);
            Ray3f rayYX(lightPointSampled, nyx);
            Intersection itsOnSource;
            Intersection itsOnObject;
            if ((!scene->rayIntersect(rayXY, itsOnSource) || !itsOnSource.p.isApprox(lightPointSampled,0.00001)) || (!scene->rayIntersect(rayYX, itsOnObject)  || !itsOnObject.p.isApprox( its.p,0.00001))) {
                return Color3f(0.f);
            }

            Normal3f wo = its.shFrame.toLocal(-ray.d);
            Normal3f wi = its.shFrame.toLocal(-nyx);
            Color3f pOrf;
            Color3f L = Le;
            float scatteringPDf=0.f;
            if ( inMedium) {
                Color3f Tr(1.f);
                int raymarchsteps=n;
                if (medium->isHomo1())
                    raymarchsteps=1;
                rayMarch(itsOnSource,medium->sigma_s+medium->sigma_a, raymarchsteps, rayXY, scene, Color3f(0.f), Tr,medium->isHomo1());
                pOrf = medium->p(wi, wo);
                scatteringPDf = pOrf.x();
                L*= medium->sigma_s;
                L *= Tr;
            } else {
                const BSDF *bsdf = its.mesh->getBSDF();
                float geoTerm = std::abs(nx.dot(-nyx) * ny.dot(nyx));
                pOrf = bsdf->eval(BSDFQueryRecord(wi, wo, ESolidAngle));
                scatteringPDf = bsdf->pdf(BSDFQueryRecord(wi, wo, ESolidAngle));
                if (scatteringPDf == 0.f || pOrf.isZero())
                    return Color3f(0.f);

                L *= geoTerm;
            }
    L *= pOrf;
    L *= lightPointSampledPDF / (lightPointSampledPDF + scatteringPDf) / lightPointSampledPDF;
    L /= 1 / emitterCount;
                return L;

        }


    };




NORI_REGISTER_CLASS(VolPath, "vol_path");
NORI_NAMESPACE_END