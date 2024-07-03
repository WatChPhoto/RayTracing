#ifndef CONE_H
#define CONE_H
//==============================================================================================
// Originally written in 2016 by Peter Shirley <ptrshrl@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related and
// neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public Domain Dedication
// along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//==============================================================================================

#include "hittable.h"


class cone : public hittable {
public:
  cone(const point3& C, const point3 &V, double radius, double height, shared_ptr<material> mat)
    : C1(C), V1(V), radius(fmax(0,radius)), height(fmax(0,height)), mat(mat), is_moving(false) {
    //Not optimized, just finding bounding box of sphere containing the cone.
    //can use "On the geometry of the smallest circle enclosing a finite set of points" to find smallest bounding sphere and find bounding box of that sphere.

    length = std::sqrt(radius*radius + height*height);//
    auto lvec = vec3(length, length, length); 
    bbox = aabb(C - lvec, C + lvec); //box containing sphere located at C and radius of side length.
  }

  //Moving Cone
  cone(const point3& C1, const point3& C2, const point3 &V1, const point3 &V2, double radius, double height, shared_ptr<material> mat)
    : C1(C1), V1(V1), radius(fmax(0,radius)), height(fmax(0,height)), mat(mat), is_moving(true) {
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
    double base = height;
    double hypoteneus = std::sqrt(base*base + radius*radius);
    double cos_theta = base/hypoteneus;
    
    point3 D = r.direction();//unit_vector(r.direction());
    point3 O = r.origin();
    point3 CO = O-C;
    
    double a = dot(D,V)*dot(D,V) - dot(D,D)*cos_theta*cos_theta;
    double b = 2*( (dot(D,V)*dot(CO,V)) - (dot(D, CO)*cos_theta*cos_theta) );
    double c = ( dot(CO,V)*dot(CO,V) ) - ( dot(CO,CO)*cos_theta*cos_theta );
    
    if(a<0){
      a = -a;
      b = -b;
      c = -c;
    }
    
    double h = -b/2;
    
    double discriminant = h*h - a*c;
    //std::clog<<"a is = "<<a<<std::endl;
    if (discriminant<0){
      return false;
    }
    
    auto sqrtd = sqrt(discriminant);
    
    // Find the nearest root that lies in the acceptable range.
    auto root = (h - sqrtd) / a;
    if (!ray_t.surrounds(root)) {
      root = (h + sqrtd) / a;
      if (!ray_t.surrounds(root))
	return false;
    }

    vec3 P = r.at(root);
    vec3 cp = P - C;
    float ht = dot(cp, V);
    if (ht < 0. || ht > height) return false;
    
    vec3 n = cp - (cp.length()/cos_theta)*V; 
    vec3 outward_normal = unit_vector(n); 

    rec.t = root;
    rec.p = P;//r.at(rec.t);
    rec.set_face_normal(r, outward_normal);
    rec.mat = mat;
    get_cone_uv(cp/length, rec.u, rec.v);
    return true;
  }

  aabb bounding_box() const override { return bbox; }
  
private:
  point3 C1;
  point3 V1;
  double radius;
  double height;
  double length;
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
