#ifndef CONE_H
#define CONE_H


#include "hittable.h"
#include "onb.h"


class cone : public hittable {
  public:
    // Stationary Cone
    cone(const point3& C, const vec3& V, double radius, interval displayedSection, shared_ptr<material> mat)
      : C1(C), V1(V), radius(fmax(0,radius)), displayedSection(displayedSection), mat(mat) {

        // ensures that there are no negative values for maximum and minimum height of the cone, replaces with 0 if there are
        this->displayedSection.max = fmax(0, displayedSection.max);
        this->displayedSection.min = fmax(0, displayedSection.min);

        
        height = displayedSection.max;

        // bounding box

        //Not optimized, just finding bounding box of sphere containing the cone.
        length = std::sqrt(radius*radius + height*height);
        auto lvec = vec3(length, length, length); 
        bbox = aabb(C - lvec, C + lvec); //box containing sphere located at C and radius of side length.


        // // not correct!!
        // double es_radius = std::sqrt(radius*radius + height*height)/2; // radius of the sphere enclosing the cone
        // auto rvec = vec3(es_radius, es_radius, es_radius);
        // point3 en_C = C1 + V * es_radius; // center point of the sphere enclosing the cone
        // bbox = aabb(en_C - rvec, en_C + rvec);


      }


    // Moving Cone
    cone(const point3& C1, const point3& C2, const point3 &V1, const point3 &V2, double radius, interval displayedSection, shared_ptr<material> mat)
        : C1(C1), V1(V1), radius(fmax(0,radius)), height(fmax(0,height)), mat(mat), is_moving(true) {

            // ensures that there are no negative values for maximum and minimum height of the cone, replaces with 0 if there are
            this->displayedSection.max = fmax(0, displayedSection.max);
            this->displayedSection.min = fmax(0, displayedSection.min);
            
            height = displayedSection.max;


            //Not optimized, just finding bounding box of sphere containing the cone.
            length = std::sqrt(radius*radius + height*height);//
            auto lvec = vec3(length, length, length); 
            aabb box1 = aabb(C1 - lvec, C1 + lvec); 
            aabb box2 = aabb(C2 - lvec, C2 + lvec);
            bbox = aabb(box1, box2);
            C_vec = C2-C1;
            V_vec = V2-V1;
    }


    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        point3 C = is_moving ? cone_C(r.time()):C1;
        point3 V = is_moving ? cone_V(r.time()):V1;
        double hypoteneus = std::sqrt(height*height + radius*radius);
        double cos_theta = height/hypoteneus;

        point3 D = r.direction();
        point3 O = r.origin();
        point3 CO = O-C;

        double a = dot(D,V)*dot(D,V) - dot(D, D)*cos_theta*cos_theta;
        double b = 2*( (dot(D,V)*dot(CO,V)) - (dot(D, CO)*cos_theta*cos_theta) );
        double c = ( dot(CO,V)*dot(CO,V) ) - ( dot(CO,CO)*cos_theta*cos_theta );

        if (a < 0) {
          a = -a;
          b = -b;
          c = -c;
        }


        double discriminant = b*b - 4*a*c;

        if (discriminant<0){
            return false;
        }

        double det = std::sqrt(discriminant);


        auto root = (-b-det)/(2*a);
        if (!ray_t.surrounds(root) || !displayedSection.contains(dot((r.at(root) - C), V))) {
            root = (-b+det)/(2*a);
            if (!ray_t.surrounds(root) || !displayedSection.contains(dot((r.at(root) - C), V)))
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);

        vec3 cp = rec.p - C;
        vec3 outward_normal = unit_vector(cp - cp.length()*unit_vector(V)/ cos_theta);

        rec.set_face_normal(r, outward_normal);
        rec.mat = mat;
        get_cone_uv(cp/length, rec.u, rec.v);   //*** */

        return true;
    }

    aabb bounding_box() const override { return bbox; }



    //convert for cone
    double pdf_value(const point3& origin, const vec3& direction) const override {
        // This method only works for stationary spheres.

        hit_record rec;
        if (!this->hit(ray(origin, direction), interval(0.001, infinity), rec))
            return 0;

        auto cos_theta_max = sqrt(1 - radius*radius/(center1 - origin).length_squared());
        auto solid_angle = 2*pi*(1-cos_theta_max);

        return  1 / solid_angle;
    }

    //convert for cone
    vec3 random(const point3& origin) const override {
        vec3 direction = center1 - origin;
        auto distance_squared = direction.length_squared();
        onb uvw;
        uvw.build_from_w(direction);
        return uvw.local(random_to_sphere(radius, distance_squared));
    }


  private:
    point3 C1;   //standin from center in sphere, represents tip of cone
    vec3 V1;     //"direction of increasing radius"
    interval displayedSection; //interval from the minimum and maximum rendered section of the cone
    double length;
    double height;
    double radius;
    shared_ptr<material> mat;
    bool is_moving;
    vec3 C_vec;
    vec3 V_vec;
    aabb bbox;

    point3 cone_C(double time) const {
        // Linearly interpolate from C1 to C2 according to time, where t=0 yields
        // C1, and t=1 yields C2.
        return C1 + time*C_vec;
    }

    point3 cone_V(double time) const {
        // Linearly interpolate from V1 to V2 according to time, where t=0 yields
        // V1, and t=1 yields V2.
        return V1 + time*V_vec;
    }

    
    static void get_cone_uv(const point3& p, double& u, double& v) {
        //texture with concentric circles centered at center of image map in the following way.
        //Center maps to C of cone and outer circle map  away from the C.
        u = p.x()+0.5;
        v = p.z()+0.5;
    }
};

#endif