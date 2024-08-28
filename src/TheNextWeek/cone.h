#ifndef CONE_H
#define CONE_H


#include "hittable.h"

//inclusions for plane method for bounding box
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
// #include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cmath>


class cone : public hittable {
  public:
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



        // plane method for bounding box

        // double hypoteneus = std::sqrt(height*height + radius*radius);
        // double theta = acos(height/hypoteneus);

        // double max_point_in_x_axis = fmax(find_extreme_pt(glm::vec3(1,0,0), C, V, theta, displayedSection.max),
        //                             find_extreme_pt(glm::vec3(1,0,0), C, V, theta, displayedSection.min));
        // double max_point_in_y_axis = fmax(find_extreme_pt(glm::vec3(0,1,0), C, V, theta, displayedSection.max),
        //                             find_extreme_pt(glm::vec3(0,1,0), C, V, theta, displayedSection.min));
        // double max_point_in_z_axis = fmax(find_extreme_pt(glm::vec3(0,0,1), C, V, theta, displayedSection.max),
        //                             find_extreme_pt(glm::vec3(0,0,1), C, V, theta, displayedSection.min));

        // double min_point_in_x_axis = fmax(find_extreme_pt(glm::vec3(-1,0,0), C, V, theta, displayedSection.max),
        //                             find_extreme_pt(glm::vec3(-1,0,0), C, V, theta, displayedSection.min));
        // double min_point_in_y_axis = fmax(find_extreme_pt(glm::vec3(0,1,0), C, V, theta, displayedSection.max),
        //                             find_extreme_pt(glm::vec3(0,-1,0), C, V, theta, displayedSection.min));
        // double min_point_in_z_axis = fmax(find_extreme_pt(glm::vec3(0,0,1), C, V, theta, displayedSection.max),
        //                             find_extreme_pt(glm::vec3(0,0,-1), C, V, theta, displayedSection.min));

        // bbox.x = interval(min_point_in_x_axis, max_point_in_x_axis);
        // bbox.y = interval(min_point_in_y_axis, max_point_in_y_axis);
        // bbox.z = interval(min_point_in_z_axis, max_point_in_z_axis);
      }


      double find_extreme_pt(glm::vec3 dir, vec3 C1, vec3 V1, float theta, float height){
      //
        glm::vec3 gC;
        glm::vec3 gV;
        
        gC.x = C1.x();
        gC.y = C1.y();
        gC.z = C1.z();

        gV.x = V1.x();
        gV.y = V1.y();
        gV.z = V1.z();
      //

        float r = height*tan(theta);
        
        float th_y = asin(gV.x);
        float th_x = atan2(-gV.y, gV.z);

        float sinx = sin(th_x);
        float cosx = cos(th_x);
        float siny = sin(th_y);
        float cosy = cos(th_y);
        
        glm::mat3 R(cosy, sinx*siny, -cosx*siny,
              0,    cosx,       sinx,
              siny, -sinx*cosy, cosx*cosy); //glm stores column first, so it's actually storing a transpose
        
        
        //model space to world space
        glm::mat3 R_t = glm::transpose(R); //Inverse of R(Inverse = Transpose for orthonormal matrix).
        //  std::cout<<R_t<<std::endl;
        glm::vec3 dir_t = R_t*dir; //Direction X vector in model space.
        glm::vec3 dir_t_ll = glm::dvec3(dir_t.x, dir_t.y,0.0f); //parallel component of dir in model space
        dir_t_ll = glm::normalize(dir_t_ll); //normalized dir_t_ll
        glm::vec3 P_t = r*dir_t_ll; //intersection point
        glm::vec3 P = R*P_t + gC + height*gV;

        //std::cout<<"( x,y,z ) = "<<P.x<<", "<<P.y<<", "<<P.z<<std::endl;
        
        return double(glm::dot(P,dir));
      }


    //Moving Cone
    cone(const point3& C1, const point3& C2, const point3 &V1, const point3 &V2, double radius, interval displayedSection, shared_ptr<material> mat)
        : C1(C1), V1(V1), radius(fmax(0,radius)), height(fmax(0,height)), mat(mat), is_moving(true) {

            // ensures that there are no negative values for maximum and minimum height of the cone, replaces with 0 if there are
            this->displayedSection.max = fmax(0, displayedSection.max);
            this->displayedSection.min = fmax(0, displayedSection.min);
            
            height = displayedSection.max;


            //Not optimized, just finding bounding box of sphere containing the cone.
            length = std::sqrt(radius*radius + height*height);
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
        get_cone_uv(cp/length, rec.u, rec.v);

        return true;
    }

    aabb bounding_box() const override { return bbox; }

  private:
    point3 C1;   //standin from center in sphere, represents tip of cone
    vec3 V1;     //"direction of increasing radius"
    double radius;
    interval displayedSection; //interval from the minimum and maximum rendered section of the cone
    shared_ptr<material> mat;
    double height;
    double length;
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