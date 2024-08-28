#ifndef CONE_H
#define CONE_H

#include "hittable.h"


class cone : public hittable {
public:
  cone(const point3& C, const point3 &V, double radius, double height, shared_ptr<material> mat)
    : C(C), V(V), radius(fmax(0,radius)), height(fmax(0,height)), mat(mat) {}
  
  bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
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

    if (discriminant<0){
      return false;
    }
    
    auto sqrtd = sqrt(discriminant);
    
    // Find the nearest root that lies in the acceptable range.
    auto root = (h - sqrtd) / a;
    vec3 P = r.at(root);
    vec3 cp = P - C;
    float ht = dot(cp, V);
    bool is_out = ht < 0. || ht > height;

    if (!ray_t.surrounds(root) || is_out) {
      root = (h + sqrtd) / a;
      P = r.at(root);
      cp = P - C;
      ht = dot(cp, V);
      is_out = ht < 0. || ht > height;
      if (!ray_t.surrounds(root) || is_out)
	return false;
    }

    
    vec3 n = cp - (cp.length()/cos_theta)*V; 
    vec3 outward_normal = unit_vector(n); 

    rec.t = root;
    rec.p = P;//r.at(rec.t);
    rec.set_face_normal(r, outward_normal);
    rec.mat = mat;
    
    return true;
  }
  
private:
  point3 C;
  point3 V;
  double radius;
  double height;
  shared_ptr<material> mat;
};


#endif
