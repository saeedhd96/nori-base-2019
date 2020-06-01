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

    class PathMats : public Integrator {
    public:
        PathMats(const PropertyList &props) {

        }



        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);


            int k = 0;
            float q = 1.f;
            float etas = 1.f;
            Color3f alpha(1.f);
            Color3f L(0.f);
            Normal3f w_i(ray.d.normalized());

            while (true) {
                w_i = its.shFrame.toLocal(-w_i);

                if (its.mesh->isEmitter()){
                    L += its.mesh->getEmitter()->getRadiance()*alpha;
                }

                float x = sampler->next1D();
                if (x < q) {

                    //Sample outgoing ray and find its intersection with the scene
                    BSDFQueryRecord query(w_i, NULL, ESolidAngle);
                    Color3f alphaa = its.mesh->getBSDF()->sample(query, sampler->next2D());
                    w_i = its.shFrame.toWorld(query.wo);
                    Ray3f newRay(its.p,w_i);
                    scene->rayIntersect(newRay,its);


                    alpha *= alphaa/q;
                    k++;
                    float maxComponentThroughput = alpha.maxCoeff();
                    etas *= query.eta;
                    float xx= maxComponentThroughput*etas*etas;
                    q= 0.99< xx?0.99:xx;


                }else {

                    return L;
                }
                }
        }



        std::string toString() const {
            return "PathMatsIntegrator[]";
        }


    };
    NORI_REGISTER_CLASS(PathMats, "path_mats");
NORI_NAMESPACE_END
