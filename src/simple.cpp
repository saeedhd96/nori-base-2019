#include <nori/integrator.h>
#include <nori/scene.h>
#include <math.h>


NORI_NAMESPACE_BEGIN

    class SimpleIntegrator : public Integrator {
    public:
        SimpleIntegrator(const PropertyList &props) {
            pointLight = props.getPoint("position");
            color = props.getColor("energy");
        }

        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
            /* Find the surface that is visible in the requested direction */
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);

            /* Return the component-wise absolute
               value of the shading normal as a color */

            Vector3f diff(pointLight -its.p);
            Normal3f n = its.shFrame.n;

            float dotProd = diff.dot(n);
            float diffSize = diff.x()*diff.x()+diff.y()*diff.y()+diff.z()*diff.z();
            float nSize = n.x()*n.x()+n.y()*n.y()+n.z()*n.z();

            float raidanceCoef = dotProd/(sqrt(diffSize)*nSize);

            raidanceCoef = std::max((float)0,raidanceCoef);
            raidanceCoef /= (4*M_PI*M_PI*diffSize);

            Ray3f ray_(its.p, diff);
            float v = raidanceCoef;
            if (scene->rayIntersect(ray_)) {
                v = 0;
            }

            return Color3f(color.x()*v, color.y()*v, v*color.z());
        }

        std::string toString() const {
            return "SimpleIntegrator[]";
        }


    protected:
        Color3f color;
        Point3f pointLight;

    };
    NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END
